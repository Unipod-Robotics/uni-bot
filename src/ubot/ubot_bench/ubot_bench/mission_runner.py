"""mission_runner: navigation phase. AMCL localises on the stack's own SLAM map; Nav2 drives the
world's goal sequence. Success is judged on ground truth, never on the robot's belief.

Frames: goals are defined in the Gazebo world frame. The SLAM map frame is where the robot
started mapping (its spawn pose), so goals are sent in the map frame as T_spawn^-1 * goal. A
mis-built map therefore misplaces goals, and that error is part of what is measured.

Per goal (result.json 'goals'):
  nav_status        Nav2 result (SUCCEEDED / FAILED / CANCELED / TIMEOUT)
  success           ground truth within 0.25 m and 0.30 rad of the goal when Nav2 finished
  time_s            sim time from dispatch to Nav2 result
  gt_path_m         ground-truth distance driven during the leg
  start_xy          ground-truth start of the leg (for the SPL shortest path)
  final_err_m / final_err_rad   ground-truth error at the end of the leg
  recoveries        Nav2 feedback number_of_recoveries at the end of the leg
  contacts          chassis contacts during the leg
Trajectories: gt.tum (world) and amcl.tum (map -> base_footprint, the localisation estimate).

Usage: ros2 run ubot_bench mission_runner --world arena_5x5 --out DIR
"""
import argparse
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from ubot_bench.common import (TumWriter, load_mission, quat_z, stamp_s, wait_for, wrap,
                               write_json, yaw_of)

GOAL_TOL_M = 0.25
GOAL_TOL_RAD = 0.30
V_NOMINAL = 0.25     # m/s, nav2_bench.yaml max_vel_x


class Recorder(Node):
    def __init__(self, out):
        super().__init__('mission_recorder',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.lock = threading.Lock()
        self.gt = None
        self.gt_n = 0
        self.path_m = 0.0
        self.contacts = []
        self.gt_tum = TumWriter(os.path.join(out, 'gt.tum'))
        self.amcl_tum = TumWriter(os.path.join(out, 'amcl.tum'))
        self.pending = []
        self.tf = Buffer(cache_time=Duration(seconds=120))
        self.tfl = TransformListener(self.tf, self)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt, 50)
        self.create_subscription(String, '/ground_truth/contacts', self.on_contact, 50)

    def on_gt(self, m):
        t = stamp_s(m.header.stamp)
        p = m.pose.pose
        yaw = yaw_of(p.orientation)
        with self.lock:
            if self.gt is not None:
                self.path_m += math.hypot(p.position.x - self.gt[1], p.position.y - self.gt[2])
            self.gt = (t, p.position.x, p.position.y, yaw)
            self.gt_tum.add(t, p.position.x, p.position.y, yaw)
            self.gt_n += 1
            if self.gt_n % 5 == 0:
                self.pending.append(m.header.stamp)

    def on_contact(self, m):
        with self.lock:
            self.contacts.append(m.data)

    def flush(self):
        with self.lock:
            pending, self.pending = self.pending, []
        keep = []
        now = self.get_clock().now().nanoseconds * 1e-9
        for st in pending:
            try:
                tr = self.tf.lookup_transform('map', 'base_footprint', Time.from_msg(st))
                t = tr.transform.translation
                self.amcl_tum.add(stamp_s(st), t.x, t.y, yaw_of(tr.transform.rotation))
            except Exception:
                if now - stamp_s(st) < 2.0:
                    keep.append(st)
        with self.lock:
            self.pending = keep + self.pending

    def snapshot(self):
        with self.lock:
            return self.gt, self.path_m, len(self.contacts)


def to_map(goal, spawn):
    """World-frame goal (x, y, yaw) -> SLAM map frame (origin = spawn pose)."""
    sx, sy, syaw = spawn
    dx, dy = goal[0] - sx, goal[1] - sy
    c, s = math.cos(-syaw), math.sin(-syaw)
    return c * dx - s * dy, s * dx + c * dy, wrap(goal[2] - syaw)


def pose_msg(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = quat_z(yaw)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--startup-timeout', type=float, default=180.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    mission = load_mission(a.world)
    spawn = (mission['spawn']['x'], mission['spawn']['y'], mission['spawn']['yaw'])
    goals = mission['goals']

    rclpy.init()
    rec = Recorder(a.out)
    nav = BasicNavigator(namespace='')
    nav.set_parameters([Parameter('use_sim_time', value=True)])
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(rec)
    threading.Thread(target=ex.spin, daemon=True).start()
    result = {'phase': 'navigation', 'world': a.world, 'goals': []}
    wall0 = time.time()
    try:
        if not wait_for(lambda: rec.gt is not None, a.startup_timeout):
            result['status'] = 'startup_timeout'
            return
        time.sleep(3.0)
        # The robot starts exactly at the spawn = SLAM map origin.
        nav.setInitialPose(pose_msg(nav, 0.0, 0.0, 0.0))
        # waitUntilNav2Active blocks forever if a lifecycle transition was lost (seen under heavy
        # machine load: map_server's change_state response timed out, AMCL never activated).
        # Bound it; a startup failure is an infrastructure failure the orchestrator retries.
        ready = threading.Event()
        threading.Thread(target=lambda: (nav.waitUntilNav2Active(localizer='amcl'),
                                         ready.set()), daemon=True).start()
        if not ready.wait(timeout=a.startup_timeout):
            result['status'] = 'startup_timeout'
            return
        time.sleep(2.0)
        t_start = rec.get_clock().now().nanoseconds * 1e-9
        for i, g in enumerate(goals):
            gx, gy, gyaw = to_map(g, spawn)
            gt0, path0, c0 = rec.snapshot()
            straight = math.hypot(g[0] - gt0[1], g[1] - gt0[2])
            budget = max(60.0, 4.0 * straight / V_NOMINAL + 30.0)   # sim seconds
            t0 = rec.get_clock().now().nanoseconds * 1e-9
            nav.goToPose(pose_msg(nav, gx, gy, gyaw))
            recoveries = 0
            status = None
            while not nav.isTaskComplete():
                fb = nav.getFeedback()
                if fb is not None:
                    recoveries = max(recoveries, int(fb.number_of_recoveries))
                rec.flush()
                if rec.get_clock().now().nanoseconds * 1e-9 - t0 > budget:
                    nav.cancelTask()
                    status = 'TIMEOUT'
                    break
                time.sleep(0.1)
            res = nav.getResult()
            status = status or {TaskResult.SUCCEEDED: 'SUCCEEDED', TaskResult.FAILED: 'FAILED',
                                TaskResult.CANCELED: 'CANCELED'}.get(res, str(res))
            time.sleep(0.5)
            gt1, path1, c1 = rec.snapshot()
            err_m = math.hypot(g[0] - gt1[1], g[1] - gt1[2])
            err_r = abs(wrap(g[2] - gt1[3]))
            result['goals'].append({
                'index': i, 'goal': g, 'nav_status': status,
                'success': bool(err_m <= GOAL_TOL_M and err_r <= GOAL_TOL_RAD),
                'time_s': rec.get_clock().now().nanoseconds * 1e-9 - t0,
                'gt_path_m': path1 - path0, 'start_xy': [gt0[1], gt0[2]],
                'final_err_m': err_m, 'final_err_rad': err_r,
                'recoveries': recoveries, 'contacts': c1 - c0, 'budget_s': budget})
            print(f"goal {i + 1}/{len(goals)}: {status} success={result['goals'][-1]['success']} "
                  f'err={err_m:.2f} m t={result["goals"][-1]["time_s"]:.0f} s', flush=True)
        result['status'] = 'ok'
        result['sim_duration_s'] = rec.get_clock().now().nanoseconds * 1e-9 - t_start
        result['wall_duration_s'] = time.time() - wall0
    finally:
        rec.flush()
        with rec.lock:
            result['contacts'] = list(rec.contacts)
            result['samples'] = {'gt': rec.gt_tum.n, 'amcl': rec.amcl_tum.n}
        rec.gt_tum.close()
        rec.amcl_tum.close()
        write_json(os.path.join(a.out, 'result.json'), result)
        ex.shutdown()
        rec.destroy_node()
        nav.destroy_node()
        rclpy.try_shutdown()
        n = len(result['goals'])
        print(f"navigation {result.get('status')}: "
              f"{sum(g['success'] for g in result['goals'])}/{n} goals")


if __name__ == '__main__':
    main()
