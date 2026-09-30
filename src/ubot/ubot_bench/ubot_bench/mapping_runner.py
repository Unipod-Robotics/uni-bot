"""mapping_runner: drive the world's mapping route with a ground-truth pure-pursuit follower
while slam_toolbox builds the map, then save the map. Every stack follows the identical path,
so differences in the map and in SLAM accuracy come from the sensor stack alone.

Outputs in --out:
  gt.tum     ground truth base_footprint pose (Gazebo world frame), at Gazebo pose rate
  odom.tum   EKF estimate /odometry/filtered (odom frame) = the odometry-only (B0) baseline
  slam.tum   SLAM estimate map -> base_footprint (TF), sampled at every 5th ground-truth stamp
  map.yaml/.pgm, map.posegraph/.data   slam_toolbox map and serialized pose graph
  result.json  status, timings, route and driven length, contacts

Usage (normally started by the orchestrator next to bench_sim.launch.py phase:=mapping):
  ros2 run ubot_bench mapping_runner --world arena_5x5 --out DIR
"""
import argparse
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from ubot_bench.common import (TumWriter, load_mission, stamp_s, wait_for, wrap, write_json,
                               yaw_of)

SPEED = 0.20          # m/s along the route
LOOKAHEAD = 0.40      # m
W_MAX = 1.0           # rad/s
ROTATE_FIRST = math.radians(60)


class Mapper(Node):
    def __init__(self, out):
        super().__init__('mapping_runner',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.out = out
        self.lock = threading.Lock()
        self.gt = None
        self.gt_n = 0
        self.contacts = []
        self.gt_tum = TumWriter(os.path.join(out, 'gt.tum'))
        self.odom_tum = TumWriter(os.path.join(out, 'odom.tum'))
        self.slam_tum = TumWriter(os.path.join(out, 'slam.tum'))
        self.tf = Buffer(cache_time=Duration(seconds=120))
        self.tfl = TransformListener(self.tf, self)
        self.pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt, 50)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 50)
        self.create_subscription(String, '/ground_truth/contacts', self.on_contact, 50)
        self.pending_slam = []

    def on_gt(self, m):
        t = stamp_s(m.header.stamp)
        p = m.pose.pose
        yaw = yaw_of(p.orientation)
        with self.lock:
            self.gt = (t, p.position.x, p.position.y, yaw)
            self.gt_tum.add(t, p.position.x, p.position.y, yaw)
            self.gt_n += 1
            if self.gt_n % 5 == 0:
                self.pending_slam.append(m.header.stamp)

    def on_odom(self, m):
        p = m.pose.pose
        with self.lock:
            self.odom_tum.add(stamp_s(m.header.stamp), p.position.x, p.position.y,
                              yaw_of(p.orientation))

    def on_contact(self, m):
        with self.lock:
            self.contacts.append(m.data)

    def flush_slam(self):
        """Look up map->base_footprint at queued ground-truth stamps (once TF has caught up)."""
        with self.lock:
            pending, self.pending_slam = self.pending_slam, []
        keep = []
        for st in pending:
            try:
                tr = self.tf.lookup_transform('map', 'base_footprint', Time.from_msg(st))
                t = tr.transform.translation
                self.slam_tum.add(stamp_s(st), t.x, t.y, yaw_of(tr.transform.rotation))
            except Exception:
                if self.get_clock().now().nanoseconds * 1e-9 - stamp_s(st) < 2.0:
                    keep.append(st)          # TF not there yet; retry for up to 2 s
        with self.lock:
            self.pending_slam = keep + self.pending_slam

    def send(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_footprint'
        m.twist.linear.x, m.twist.angular.z = float(v), float(w)
        self.pub.publish(m)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9


def follow(node, route, timeout_s):
    """Pure pursuit along route (N x 2, world frame) on ground truth. Returns status string."""
    seg = np.linalg.norm(np.diff(route, axis=0), axis=1)
    s_cum = np.concatenate([[0.0], np.cumsum(seg)])
    idx = 0
    t_end = node.now_s() + timeout_s
    while rclpy.ok():
        if node.now_s() > t_end:
            node.send(0.0, 0.0)
            return 'timeout'
        with node.lock:
            _, x, y, yaw = node.gt
        # progress: nearest route point within the next 2 m of arc (never goes backwards)
        ahead = np.searchsorted(s_cum, s_cum[idx] + 2.0)
        window = route[idx:max(ahead, idx + 1) + 1]
        idx += int(np.argmin(np.hypot(window[:, 0] - x, window[:, 1] - y)))
        if idx >= len(route) - 1 and math.hypot(route[-1, 0] - x, route[-1, 1] - y) < 0.15:
            node.send(0.0, 0.0)
            return 'ok'
        j = min(int(np.searchsorted(s_cum, s_cum[idx] + LOOKAHEAD)), len(route) - 1)
        tx, ty = route[j]
        dx, dy = tx - x, ty - y
        lx = math.cos(yaw) * dx + math.sin(yaw) * dy
        ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
        alpha = math.atan2(ly, lx)
        if abs(alpha) > ROTATE_FIRST:
            v, w = 0.0, W_MAX * np.sign(alpha) * 0.6
        else:
            d = max(math.hypot(lx, ly), 1e-3)
            v = SPEED
            w = float(np.clip(2 * v * ly / (d * d), -W_MAX, W_MAX))
            if abs(w) > 0.8 * W_MAX:
                v *= 0.5
        node.send(v, w)
        node.flush_slam()
        time.sleep(0.05)
    return 'interrupted'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--startup-timeout', type=float, default=240.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    mission = load_mission(a.world)
    route = np.asarray(mission['mapping_route'], dtype=float)
    length = float(np.sum(np.linalg.norm(np.diff(route, axis=0), axis=1)))

    rclpy.init()
    node = Mapper(a.out)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    result = {'phase': 'mapping', 'world': a.world, 'route_length_m': length}
    wall0 = time.time()
    try:
        ready = wait_for(lambda: node.gt is not None and node.pub.get_subscription_count() > 0
                         and node.tf.can_transform('map', 'base_footprint', Time()),
                         a.startup_timeout)
        if not ready:
            result['status'] = 'startup_timeout'
            return
        time.sleep(3.0)                          # let the robot settle after the spawn drop
        t0 = node.now_s()
        status = follow(node, route, timeout_s=2.5 * length / SPEED + 60.0)
        node.send(0.0, 0.0)
        t1 = node.now_s()
        time.sleep(2.0)
        node.flush_slam()
        result.update(status=status, sim_duration_s=t1 - t0, wall_duration_s=time.time() - wall0)
        result.update(save_map(node, a.out))
    finally:
        with node.lock:
            result['contacts'] = list(node.contacts)
            result['samples'] = {'gt': node.gt_tum.n, 'odom': node.odom_tum.n,
                                 'slam': node.slam_tum.n}
        for w in (node.gt_tum, node.odom_tum, node.slam_tum):
            w.close()
        write_json(os.path.join(a.out, 'result.json'), result)
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
        print(f"mapping {result.get('status')}: {result}")


def save_map(node, out):
    """slam_toolbox map (pgm/yaml) and serialized pose graph."""
    from slam_toolbox.srv import SaveMap, SerializePoseGraph
    from std_msgs.msg import String as Str
    res = {}
    cli = node.create_client(SaveMap, '/slam_toolbox/save_map')
    if cli.wait_for_service(timeout_sec=30.0):
        fut = cli.call_async(SaveMap.Request(name=Str(data=os.path.join(out, 'map'))))
        res['map_saved'] = wait_for(fut.done, 60.0) and fut.result() is not None
    cli2 = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
    if cli2.wait_for_service(timeout_sec=10.0):
        fut = cli2.call_async(SerializePoseGraph.Request(filename=os.path.join(out, 'map')))
        res['posegraph_saved'] = wait_for(fut.done, 60.0) and fut.result() is not None
    res['map_saved'] = res.get('map_saved', False) and os.path.exists(
        os.path.join(out, 'map.yaml'))
    return res


if __name__ == '__main__':
    main()
