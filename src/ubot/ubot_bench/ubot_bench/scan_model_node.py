"""scan_model: clean simulated scan -> datasheet-noise scan.

Subscribes  /scan_raw (sensor_msgs/LaserScan, Gazebo gpu_lidar with no noise, or the
            depthimage_to_laserscan output for the depth stack)
Publishes   /scan

Parameters
  profile       sensor_profiles.yaml key (e.g. lidar_ms200)
  seed          RNG seed; same seed -> same noise sequence for a given message stream
  noise         apply the datasheet noise model (false = ground-truth scan)
  dropout, outlier, noise_scale   C3 degradation (see profiles.RangeNoiseModel)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from ubot_bench.profiles import RangeNoiseModel, load_profiles


class ScanModel(Node):
    def __init__(self):
        super().__init__('scan_model')
        self.declare_parameter('profile', 'lidar_ms200')
        self.declare_parameter('seed', 0)
        self.declare_parameter('noise', True)
        self.declare_parameter('dropout', 0.0)
        self.declare_parameter('outlier', 0.0)
        self.declare_parameter('noise_scale', 1.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.profile = load_profiles()[g('profile')]
        self.model = RangeNoiseModel(self.profile, int(g('seed')), bool(g('noise')),
                                     float(g('dropout')), float(g('outlier')),
                                     float(g('noise_scale')))
        self.rmin = float(self.profile['range_min'])
        self.rmax = float(self.profile['range_max'])
        # Reliable, so both reliable (slam_toolbox) and best-effort (costmap, AMCL) subscribers match.
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(LaserScan, '/scan_raw', self.cb, qos_profile_sensor_data)
        self.get_logger().info(
            f"{self.profile['label']}: noise={g('noise')} seed={g('seed')} dropout={g('dropout')} "
            f"outlier={g('outlier')} noise_scale={g('noise_scale')}")

    def cb(self, msg):
        msg.ranges = self.model.apply(msg.ranges, self.rmin, self.rmax).tolist()
        msg.range_min = self.rmin
        msg.range_max = self.rmax
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
