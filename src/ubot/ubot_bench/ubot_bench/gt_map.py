"""gt_map: build the ground-truth occupancy map of a benchmark world at the LiDAR scan plane.

Method (documented in PROTOCOL.md, "Ground-truth maps"):
  1. Start Gazebo headless on the world's nominal condition and spawn gt_scanner, a noiseless,
     collision-free 360 deg / 0.125 deg / 30 m range sensor, at the robot's scan-plane height.
  2. Teleport the scanner to the spawn pose, take a scan at that exact pose, and integrate it
     (per-cell hit and pass counts along each beam).
  3. Candidate scan positions are the nodes of a lattice (default 0.5 m). A node becomes eligible
     once it is known free with >= clearance to anything occupied or unknown, lies inside the
     world's bounds, and is 8-connected through known-free space to the spawn. Visit eligible
     nodes in breadth-first order until none remain.
  The scanner carries three LiDARs (0.08, 0.22, 0.3627 m). Each slice is integrated separately.
  Exploration (step 3) uses the union of the slices, so the scanner never leaves through a window
  that the scan plane passes over. Outputs: <world>.yaml = the 0.3627 m scan-plane map (map-quality
  ground truth) and <world>_traversable.yaml = the union (missions, SPL, clearance).
  4. Label cells (the final maps; exploration in step 3 uses a lenient one-observation labelling):
     occupied if hits >= 2 and hits / (hits + passes) >= 0.25; free if passes >= 2
     and not occupied; unknown otherwise.

The scanner never drives, so the map is independent of any robot, controller or SLAM; the only
model is the rendered geometry. Output: ubot_worlds maps_gt/<world>.{yaml,pgm} (+ scan log).

Usage: ros2 run ubot_bench gt_map --world small_house [--lattice 0.5] [--out DIR]
"""
import argparse
import math
import os
import subprocess
import tempfile
import time
from collections import deque

import numpy as np
import yaml
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode

from ubot_bench.grid import FREE, OCC, UNKNOWN, Grid, connected_from, distance_to

SCAN_HEIGHT = 0.3627       # lidar_link height above the floor in ubot_robot.urdf.xacro
RES = 0.05
SLICES = ('z08', 'z22', 'z36')   # gt_scanner LiDAR heights 0.08 / 0.22 / 0.3627 m; z36 = scan plane


def share(pkg, *p):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(pkg), *p)


class Scanner:
    def __init__(self, world):
        self.world = world
        self.node = GzNode()
        self.last = {k: None for k in SLICES}
        for k in SLICES:
            self.node.subscribe(LaserScan, f'/gt_scanner/scan_{k}',
                                lambda msg, k=k: self.last.__setitem__(k, msg))

    def _req(self, service, req, req_type, timeout=5000):
        ok, rep = self.node.request(f'/world/{self.world}/{service}', req, req_type, Boolean,
                                    timeout)
        return ok and rep.data

    def spawn(self, x, y):
        f = EntityFactory()
        with open(share('ubot_bench', 'models', 'gt_scanner', 'model.sdf')) as fh:
            f.sdf = fh.read()
        f.name = 'gt_scanner'
        f.pose.position.x, f.pose.position.y, f.pose.position.z = x, y, SCAN_HEIGHT
        f.pose.orientation.w = 1.0
        return self._req('create', f, EntityFactory)

    @staticmethod
    def _stamp(m):
        return m.header.stamp.sec + m.header.stamp.nsec * 1e-9

    def scan_at(self, x, y, timeout=10.0):
        """Teleport, then return {slice: scan} with every scan stamped >= 3 frames after the
        teleport request.

        gpu_lidar leaves LaserScan.world_pose empty, so freshness is decided by sim time: the
        set_pose request is applied on the next iteration, and a scan rendered 3 frames (0.15 s
        at 20 Hz) after the last one seen before the request is guaranteed post-teleport.
        """
        t0 = time.time()
        while any(v is None for v in self.last.values()):
            if time.time() - t0 > timeout:
                raise TimeoutError('gt_scanner publishes no scans')
            time.sleep(0.02)
        t_req = max(self._stamp(m) for m in self.last.values())
        p = Pose()
        p.name = 'gt_scanner'
        p.position.x, p.position.y, p.position.z = x, y, SCAN_HEIGHT
        p.orientation.w = 1.0
        for attempt in range(5):        # the service occasionally times out under load
            if self._req('set_pose', p, Pose):
                break
            time.sleep(0.5)
        else:
            raise RuntimeError(f'set_pose failed 5 times at ({x:.2f}, {y:.2f})')
        while time.time() - t0 < timeout:
            snap = dict(self.last)
            if all(m is not None and self._stamp(m) >= t_req + 0.149 for m in snap.values()):
                return snap
            time.sleep(0.01)
        raise TimeoutError(f'no scan at ({x:.2f}, {y:.2f})')


class Integrator:
    def __init__(self, bounds, margin=2.0):
        xmin, xmax, ymin, ymax = bounds
        self.origin = (xmin - margin, ymin - margin)
        self.cols = int(math.ceil((xmax - xmin + 2 * margin) / RES))
        self.rows = int(math.ceil((ymax - ymin + 2 * margin) / RES))
        self.hits = np.zeros(self.rows * self.cols, dtype=np.int32)
        self.passes = np.zeros(self.rows * self.cols, dtype=np.int32)
        self.grid = Grid(np.full((self.rows, self.cols), UNKNOWN, dtype=np.int8), RES,
                         self.origin)

    def _flat(self, x, y):
        r, c = self.grid.world_to_cell(x, y)
        ok = self.grid.inside(r, c)
        return r[ok] * self.cols + c[ok]

    def add(self, scan, x, y):
        n = scan.count
        ang = scan.angle_min + np.arange(n) * scan.angle_step
        rng = np.asarray(scan.ranges, dtype=float)
        hit = np.isfinite(rng) & (rng >= scan.range_min) & (rng < scan.range_max)
        free_len = np.where(hit, rng - RES, scan.range_max)
        # Free space along every beam, sampled at half-cell spacing.
        steps = np.arange(0.0, scan.range_max, RES / 2)
        s = steps[None, :]
        keep = s < free_len[:, None]
        px = (x + s * np.cos(ang)[:, None])[keep]
        py = (y + s * np.sin(ang)[:, None])[keep]
        cells = np.unique(self._flat(px, py))
        self.passes[cells] += 1
        hx = x + rng[hit] * np.cos(ang[hit])
        hy = y + rng[hit] * np.sin(ang[hit])
        hcells = np.unique(self._flat(hx, hy))
        self.hits[hcells] += 1
        # Passes through a cell that is also hit in this scan are grazing rays, not evidence.
        self.passes[np.intersect1d(cells, hcells)] -= 1

    def label(self, strict=True):
        """strict: the final map (>= 2 observations per label). Lenient (exploration only): any
        passing beam with no hit makes a cell free, any hit makes it occupied."""
        h, p = self.hits, self.passes
        if strict:
            occ = (h >= 2) & (h >= 0.25 * (h + p))
            free = (p >= 2) & ~occ
        else:
            occ = h >= 1
            free = (p >= 1) & ~occ
        d = np.full(h.shape, UNKNOWN, dtype=np.int8)
        d[free] = FREE
        d[occ] = OCC
        self.grid.data = d.reshape(self.rows, self.cols)
        return self.grid


def union(grids):
    """Traversability: occupied if any slice is occupied; free if some slice saw it free and
    none saw it occupied; unknown otherwise."""
    d = np.full(grids[0].shape, UNKNOWN, dtype=np.int8)
    free = np.zeros(grids[0].shape, dtype=bool)
    occ = np.zeros(grids[0].shape, dtype=bool)
    for g in grids:
        free |= g.data == FREE
        occ |= g.data == OCC
    d[free & ~occ] = FREE
    d[occ] = OCC
    return Grid(d, grids[0].res, grids[0].origin)


def eligible_nodes(grid, spawn, bounds, lattice, clearance, visited):
    free = grid.data == FREE
    blocked = grid.data != FREE
    dist = distance_to(blocked, grid.res)
    r0, c0 = grid.world_to_cell(spawn[0], spawn[1])
    reach = connected_from(free, int(r0), int(c0))
    xmin, xmax, ymin, ymax = bounds
    xs = np.arange(xmin + lattice / 2, xmax, lattice)
    ys = np.arange(ymin + lattice / 2, ymax, lattice)
    out = []
    for x in xs:
        for y in ys:
            key = (round(x, 3), round(y, 3))
            if key in visited:
                continue
            r, c = grid.world_to_cell(x, y)
            if grid.inside(r, c) and reach[r, c] and dist[r, c] >= clearance:
                out.append(key)
    return out


def main():
    # SIGTERM (e.g. `timeout`, pkill) must still run the finally-block that stops Gazebo.
    import signal
    import sys
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--world', required=True)
    ap.add_argument('--lattice', type=float, default=0.5)
    ap.add_argument('--clearance', type=float, default=0.30)
    ap.add_argument('--max-scans', type=int, default=3000)
    ap.add_argument('--out', default=None, help='output dir (default: ubot_worlds source maps_gt)')
    a = ap.parse_args()

    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'gz_world', share('ubot_worlds', 'launch', 'gz_world.launch.py'))
    gz_world = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gz_world)
    with open(share('ubot_worlds', 'missions', f'{a.world}.yaml')) as f:
        mission = yaml.safe_load(f)
    bounds = mission['bounds']
    spawn = (mission['spawn']['x'], mission['spawn']['y'])
    sdf = gz_world.render_world(a.world, 'nominal')

    env = dict(os.environ)
    env.setdefault('GZ_PARTITION', f'gtmap_{a.world}_{os.getpid()}')
    os.environ['GZ_PARTITION'] = env['GZ_PARTITION']
    log = open(os.path.join(tempfile.gettempdir(), f'gt_map_{a.world}.log'), 'w')
    gz = subprocess.Popen(['gz', 'sim', '-s', '-r', '-v', '1', sdf], env=env, stdout=log,
                          stderr=subprocess.STDOUT, start_new_session=True)
    try:
        sc = Scanner(a.world)
        t0 = time.time()
        while not sc.spawn(*spawn):
            if time.time() - t0 > 120:
                raise RuntimeError('gazebo did not come up')
            time.sleep(1.0)
        integ = {k: Integrator(bounds) for k in SLICES}
        visited = {}
        skipped = []
        queue = deque([(round(spawn[0], 3), round(spawn[1], 3))])
        t_start = time.time()
        while queue and len(visited) < a.max_scans:
            node = queue.popleft()
            if node in visited:
                continue
            scan = None
            for attempt in range(3):     # rendering can stall briefly on a loaded machine
                try:
                    scan = sc.scan_at(*node, timeout=20.0)
                    break
                except TimeoutError:
                    print(f'retry {attempt + 1} at {node}')
            visited[node] = len(visited)
            if scan is None:
                skipped.append(node)
                continue
            for k in SLICES:
                integ[k].add(scan[k], *node)
            grid = union([integ[k].label(strict=False) for k in SLICES])
            # BFS order: newly eligible nodes go to the back of the queue.
            known = set(queue)
            for n in eligible_nodes(grid, spawn, bounds, a.lattice, a.clearance, visited):
                if n not in known:
                    queue.append(n)
            if len(visited) % 25 == 0 or len(visited) <= 3:
                print(f'{len(visited)} scans, {len(queue)} queued, '
                      f'{(grid.data == FREE).sum() * RES * RES:.1f} m2 free')
        plane = integ['z36'].label()
        grid = union([integ[k].label() for k in SLICES])
    finally:
        os.killpg(gz.pid, 15)
        gz.wait(timeout=20)

    out = a.out or os.path.join(os.path.expanduser('~/uni-bot/src/ubot/ubot_worlds/maps_gt'))
    os.makedirs(out, exist_ok=True)
    plane.save(os.path.join(out, f'{a.world}.yaml'))
    grid.save(os.path.join(out, f'{a.world}_traversable.yaml'))
    with open(os.path.join(out, f'{a.world}_scans.yaml'), 'w') as f:
        yaml.safe_dump({'world': a.world, 'method': 'teleported noiseless scanner',
                        'scan_height': SCAN_HEIGHT, 'lattice': a.lattice,
                        'clearance': a.clearance, 'n_scans': len(visited) - len(skipped),
                        'skipped': [[float(v) for v in k] for k in skipped],
                        'seconds': round(time.time() - t_start, 1),
                        'slices_m': [0.08, 0.22, SCAN_HEIGHT],
                        'scan_plane_free_m2': float((plane.data == FREE).sum() * RES * RES),
                        'traversable_free_m2': float((grid.data == FREE).sum() * RES * RES),
                        'positions': [[float(v) for v in k] for k in visited]}, f, default_flow_style=None)
    print(f'wrote {out}/{a.world}.yaml (+ _traversable): {len(visited)} scans, scan plane '
          f'{(plane.data == FREE).sum() * RES * RES:.1f} m2 free, traversable '
          f'{(grid.data == FREE).sum() * RES * RES:.1f} m2 free')


if __name__ == '__main__':
    main()
