"""diag: the diagnostic tests behind the findings in docs/REPRODUCE.md, part B.

Each test runs against a FRESH simulation started with bench_sim.launch.py in the arena (tests
move the robot, and ekf / bump / glass assume it starts at the spawn), reads ground truth from
Gazebo, and prints a result next to the value recorded when the finding was made.

  ros2 launch ubot_bench bench_sim.launch.py world:=arena_5x5 stack:=MS200 phase:=mapping
  ros2 run ubot_bench diag <test>

Tests
  straight  drive 3 m straight: lateral drift and per-wheel speeds          (expect 0.0 cm, equal)
  turn      command 90 deg in place: achieved yaw / commanded               (expect ~1.05)
  pivot     command 360 deg in place: circle fit of the base path,
            rotation centre in the body frame                    (expect (+0.127, 0.0) m: front axle)
  tilt      chassis roll / pitch at rest                                    (expect 0.00 / 0.00 deg)
  yawrate   integrate IMU, wheel-odometry and true yaw over still/turn/still
                                       (expect IMU still drift ~0; wheel under-reads turns ~4 %)
  ekf       drive 1 m, turn 90 deg left, drive 1 m (arena, from the spawn);
            /odometry/filtered vs truth                                     (expect yaw err < 1 deg)
  bump      reverse into the west wall (arena only): collision episodes     (expect 1 episode)
  glass     arena, condition:=glass, stack:=REF: range of a beam crossing the pane
                                              (expect the wall behind, 5.21 m, not the pane 4.87 m)
"""
import argparse
import math
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import String

from ubot_bench.common import load_mission, stamp_s, wrap, yaw_of


class Probe(Node):
    def __init__(self):
        super().__init__('diag', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.lock = threading.Lock()
        self.gt = None
        self.gt_log = []
        self.js = {}
        self.contacts = []
        self.scan = None
        self.imu_int = self.odom_int = 0.0
        self.imu_t = self.odom_t = None
        self.ekf = None
        self.pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self._gt, 50)
        self.create_subscription(JointState, '/joint_states', self._js, 50)
        self.create_subscription(String, '/ground_truth/contacts',
                                 lambda m: self.contacts.append(m.data), 50)
        self.create_subscription(LaserScan, '/scan_raw', lambda m: setattr(self, 'scan', m),
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self._imu, 100)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self._odom, 100)
        self.create_subscription(Odometry, '/odometry/filtered', self._ekf, 50)

    # ---- callbacks
    def _gt(self, m):
        p = m.pose.pose
        q = p.orientation
        with self.lock:
            self.gt = (stamp_s(m.header.stamp), p.position.x, p.position.y, yaw_of(q),
                       math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
                       math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
            self.gt_log.append(self.gt[:4])

    def _js(self, m):
        for n, v in zip(m.name, m.velocity):
            self.js.setdefault(n, []).append(v)

    def _imu(self, m):
        t = stamp_s(m.header.stamp)
        if self.imu_t is not None:
            self.imu_int += m.angular_velocity.z * (t - self.imu_t)
        self.imu_t = t

    def _odom(self, m):
        t = stamp_s(m.header.stamp)
        if self.odom_t is not None:
            self.odom_int += m.twist.twist.angular.z * (t - self.odom_t)
        self.odom_t = t

    def _ekf(self, m):
        p = m.pose.pose
        self.ekf = (p.position.x, p.position.y, yaw_of(p.orientation))

    # ---- helpers
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def drive(self, v, w, sim_s):
        end = self.now() + sim_s
        while self.now() < end:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'base_footprint'
            m.twist.linear.x, m.twist.angular.z = float(v), float(w)
            self.pub.publish(m)
            time.sleep(0.02)

    def stop(self, sim_s=1.0):
        self.drive(0.0, 0.0, sim_s)

    def wait_ready(self, timeout=240.0):
        t0 = time.time()
        while self.gt is None or self.pub.get_subscription_count() == 0:
            if time.time() - t0 > timeout:
                sys.exit('simulation not ready: start bench_sim.launch.py first')
            time.sleep(0.5)
        time.sleep(3.0)                      # settle after the spawn drop


def t_straight(n):
    t0, x0, y0, yaw0 = n.gt[:4]
    n.js.clear()
    n.drive(0.5, 0.0, 6.0)
    n.stop()
    _, x1, y1, yaw1 = n.gt[:4]
    dx, dy = x1 - x0, y1 - y0
    fwd = dx * math.cos(yaw0) + dy * math.sin(yaw0)
    lat = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
    print(f'straight: forward {fwd:.3f} m, lateral {lat * 100:+.1f} cm, '
          f'yaw {math.degrees(wrap(yaw1 - yaw0)):+.1f} deg   (expect lateral 0.0 cm)')
    for k in sorted(n.js):
        v = n.js[k][len(n.js[k]) // 4: 3 * len(n.js[k]) // 4]
        print(f'  {k:26s} {np.mean(v):7.3f} rad/s (std {np.std(v):.3f})')


def t_turn(n):
    y0 = n.gt[3]
    yaws = []
    end = n.now() + (math.pi / 2) / 1.0
    while n.now() < end:
        n.drive(0.0, 1.0, 0.05)
        yaws.append(n.gt[3])
    n.stop(1.5)
    yaws.append(n.gt[3])
    tot = math.degrees(float(np.sum(np.angle(np.exp(1j * np.diff([y0] + yaws))))))
    print(f'turn: commanded 90.0 deg, achieved {tot:.1f} deg, ratio {tot / 90:.3f} (expect ~1.05)')


def t_pivot(n):
    with n.lock:
        n.gt_log.clear()
    n.drive(0.0, 0.6, 2 * math.pi / 0.6)
    n.stop()
    L = np.array(n.gt_log)
    x, y = L[:, 1], L[:, 2]
    A = np.c_[2 * x, 2 * y, np.ones_like(x)]
    cx, cy, c = np.linalg.lstsq(A, x * x + y * y, rcond=None)[0]
    r = math.sqrt(c + cx * cx + cy * cy)
    dx, dy = cx - x[0], cy - y[0]
    c0, s0 = math.cos(L[0, 3]), math.sin(L[0, 3])
    print(f'pivot: base path radius {r:.3f} m, rotation centre in body frame '
          f'({c0 * dx + s0 * dy:+.3f}, {-s0 * dx + c0 * dy:+.3f}) m   '
          f'(expect (+0.127, 0.00): front axle, Coulomb load bias)')


def t_tilt(n):
    _, _, _, _, roll, pitch = n.gt
    print(f'tilt: roll {math.degrees(roll):+.2f} deg, pitch {math.degrees(pitch):+.2f} deg, '
          f'z {0.0:.3f}   (expect 0.00 / 0.00)')


def t_yawrate(n):
    def seg(name, w, dur):
        i0, o0, y0 = n.imu_int, n.odom_int, n.gt[3]
        n.drive(0.0, w, dur)
        dt = math.degrees(wrap(n.gt[3] - y0))
        print(f'  {name:16s} truth {dt:+7.2f}  imu {math.degrees(n.imu_int - i0):+7.2f}  '
              f'wheel {math.degrees(n.odom_int - o0):+7.2f} deg')
    print('yawrate (deg integrated per segment):')
    seg('still 5 s', 0.0, 5.0)
    seg('turn 90', 0.8, (math.pi / 2) / 0.8)
    seg('settle 1 s', 0.0, 1.0)
    seg('still 5 s', 0.0, 5.0)
    print('  expect: IMU still ~0 (BNO085 model, no bias); wheel under-reads the turn by ~4 %')


def t_ekf(n):
    if n.ekf is None:
        sys.exit('no /odometry/filtered: bench_sim runs the EKF; wait and retry')
    g0, e0 = n.gt[:4], n.ekf
    n.drive(0.4, 0.0, 2.5)                       # 1 m: from the arena spawn this L-shaped path
    n.stop(0.5)                                  # clears every obstacle
    n.drive(0.0, 0.8, (math.pi / 2) / 0.8)
    n.stop(0.5)
    n.drive(0.4, 0.0, 2.5)
    n.stop(1.5)
    g1, e1 = n.gt[:4], n.ekf

    def rel(p, o, yaw0):
        dx, dy = p[0] - o[0], p[1] - o[1]
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        return dx * c - dy * s, dx * s + dy * c
    gt_xy = rel((g1[1], g1[2]), (g0[1], g0[2]), g0[3])
    ek_xy = rel((e1[0], e1[1]), (e0[0], e0[1]), e0[2])
    yaw_err = math.degrees(abs(wrap((e1[2] - e0[2]) - (g1[3] - g0[3]))))
    print(f'ekf: truth end ({gt_xy[0]:+.3f}, {gt_xy[1]:+.3f}), EKF end ({ek_xy[0]:+.3f}, '
          f'{ek_xy[1]:+.3f}), position error {math.dist(gt_xy, ek_xy) * 100:.1f} cm, '
          f'yaw error {yaw_err:.1f} deg   (after the IMU/EKF fix: yaw error < 1 deg)')


def t_bump(n):
    n.contacts.clear()
    n.drive(-0.3, 0.0, 5.0)
    n.stop()
    print(f'bump: final x {n.gt[1]:.3f} (west wall face -2.5 + chassis 0.18), collision episodes '
          f'{len(n.contacts)} {sorted({c.split(" ", 1)[1] for c in n.contacts})}   (expect 1)')


def t_glass(n):
    t0 = time.time()
    while n.scan is None and time.time() - t0 < 30:
        time.sleep(0.5)
    if n.scan is None:
        sys.exit('no /scan_raw: use a LiDAR stack (stack:=REF)')
    m = n.scan
    pane = load_mission('arena_5x5')['conditions']['glass'][0]
    _, rx, ry, ryaw = n.gt[:4]
    lx, ly = rx + 0.0306 * math.cos(ryaw), ry + 0.0306 * math.sin(ryaw)
    tx, ty = pane['x'], pane['y'] + 0.35                       # a point on the upper pane
    a = math.atan2(ty - ly, tx - lx) - ryaw
    i = int(round((a - m.angle_min) / m.angle_increment))
    r = min(m.ranges[i - 1:i + 2])
    print(f'glass: beam at {math.degrees(a):.1f} deg crosses the pane at '
          f'{math.hypot(tx - lx, ty - ly):.2f} m; LiDAR reads {r:.2f} m   '
          f'(expect the wall behind: larger than the pane distance)')


TESTS = {'straight': t_straight, 'turn': t_turn, 'pivot': t_pivot, 'tilt': t_tilt,
         'yawrate': t_yawrate, 'ekf': t_ekf, 'bump': t_bump, 'glass': t_glass}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('test', choices=sorted(TESTS))
    a = ap.parse_args()
    rclpy.init()
    n = Probe()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    n.wait_ready()
    try:
        TESTS[a.test](n)
    finally:
        n.stop(0.5)
        sys.stdout.flush()
        # gz-transport's subscriber threads abort ("terminate called...") on a normal interpreter
        # exit, so leave immediately once the result is printed.
        import os
        os._exit(0)


if __name__ == '__main__':
    main()
