"""gt_publisher: Gazebo ground truth -> ROS.

Reads /world/<world>/pose/info (gz.msgs.Pose_V from SceneBroadcaster) over gz-transport and
publishes, stamped with the Gazebo sim time of the pose message (the same clock as /clock):

  /ground_truth/odom     nav_msgs/Odometry  pose of model 'ubot' (its base_footprint) in the
                         Gazebo world frame 'gt_world'; twist is left zero
  /ground_truth/walkers  geometry_msgs/PoseArray  poses of actors named walker_* (C2), if the
                         SceneBroadcaster reports them (analysis recomputes them from the mission
                         file anyway, since actor trajectories are deterministic)
  /ground_truth/contacts std_msgs/String  one message per chassis collision EPISODE,
                         "<sim_sec> <other collision name>" (a new episode with an object starts
                         after >= 0.5 s without contact with it), from the bench bumper sensor

No TF is published, so ground truth can never leak into the estimation stack.
"""
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseArray
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class GtPublisher(Node):
    def __init__(self):
        super().__init__('gt_publisher')
        self.declare_parameter('world', 'arena_5x5')
        self.declare_parameter('model', 'ubot')
        world = self.get_parameter('world').value
        self.model = self.get_parameter('model').value
        self.odom_pub = self.create_publisher(Odometry, '/ground_truth/odom', 50)
        self.walker_pub = self.create_publisher(PoseArray, '/ground_truth/walkers', 10)
        self.contact_pub = self.create_publisher(String, '/ground_truth/contacts', 50)
        self.last_contact = {}
        self.gz = GzNode()
        topic = f'/world/{world}/pose/info'
        if not self.gz.subscribe(Pose_V, topic, self.on_pose):
            raise RuntimeError(f'could not subscribe to {topic}')
        # The gz Contact system publishes on the sensor's scoped default topic (the URDF-level
        # <topic> is not used for contact sensors).
        self.gz.subscribe(Contacts, f'/world/{world}/model/{self.model}/link/base_footprint/'
                                    'sensor/bumper/contact', self.on_contacts)
        self.get_logger().info(f'ground truth from {topic}, model {self.model!r}')

    def on_pose(self, msg):
        stamp = Time(sec=msg.header.stamp.sec, nanosec=msg.header.stamp.nsec)
        walkers = PoseArray()
        walkers.header.stamp = stamp
        walkers.header.frame_id = 'gt_world'
        for p in msg.pose:
            if p.name == self.model:
                o = Odometry()
                o.header.stamp = stamp
                o.header.frame_id = 'gt_world'
                o.child_frame_id = 'base_footprint'
                o.pose.pose.position.x = p.position.x
                o.pose.pose.position.y = p.position.y
                o.pose.pose.position.z = p.position.z
                o.pose.pose.orientation.x = p.orientation.x
                o.pose.pose.orientation.y = p.orientation.y
                o.pose.pose.orientation.z = p.orientation.z
                o.pose.pose.orientation.w = p.orientation.w
                self.odom_pub.publish(o)
            elif p.name.startswith('walker_'):
                q = Pose()
                q.position.x, q.position.y = p.position.x, p.position.y
                q.orientation.z, q.orientation.w = p.orientation.z, p.orientation.w
                walkers.poses.append(q)
        if walkers.poses:
            self.walker_pub.publish(walkers)

    def on_contacts(self, msg):
        for c in msg.contact:
            other = c.collision2.name if 'ubot' in c.collision1.name else c.collision1.name
            if 'wheel' in other:          # the robot's own wheels never count as a collision
                continue
            t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            prev = self.last_contact.get(other)
            self.last_contact[other] = t
            if prev is None or t - prev >= 0.5:
                self.contact_pub.publish(String(data=f'{t:.3f} {other}'))


def main():
    rclpy.init()
    node = GtPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
