"""Shared runner utilities: trajectory recording, waiting, result files."""
import json
import math
import os
import time

import yaml


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def quat_z(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def stamp_s(st):
    return st.sec + st.nanosec * 1e-9


def load_mission(world):
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('ubot_worlds'), 'missions', f'{world}.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


class TumWriter:
    """TUM trajectory file: 'timestamp x y z qx qy qz qw' (planar: z = 0, roll = pitch = 0)."""

    def __init__(self, path):
        self.f = open(path, 'w')
        self.n = 0
        self.last_t = -1.0

    def add(self, t, x, y, yaw):
        if t <= self.last_t:            # keep strictly increasing timestamps
            return
        qz, qw = quat_z(yaw)
        self.f.write(f'{t:.4f} {x:.5f} {y:.5f} 0 0 0 {qz:.6f} {qw:.6f}\n')
        self.last_t = t
        self.n += 1

    def close(self):
        self.f.close()


def write_json(path, obj):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(obj, f, indent=2, default=float)
    os.replace(tmp, path)


def wait_for(pred, timeout, period=0.2):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if pred():
            return True
        time.sleep(period)
    return False
