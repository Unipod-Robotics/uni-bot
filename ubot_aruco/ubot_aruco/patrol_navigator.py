#!/usr/bin/env python3
"""
ArUco Patrol Navigator Node

Sequential patrol through ArUco markers using a state machine:
- SEARCHING: Rotate to find target marker
- APPROACH: Drive toward marker while keeping it centered
- NEXT_MARKER: Reached marker, pause, then move to next
- COMPLETE: All markers visited
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import math
from enum import Enum

from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class PatrolState(Enum):
    """Patrol state machine states."""
    SEARCHING = 1
    APPROACH = 2
    NEXT_MARKER = 3
    COMPLETE = 4


class PatrolNavigator(Node):
    """ROS 2 node for sequential ArUco marker patrol."""

    def __init__(self):
        super().__init__('patrol_navigator')

        # Declare parameters
        self.declare_parameter('marker_sequence', [0, 1, 2, 3, 4])
        self.declare_parameter('approach_distance', 0.5)      # Stop 0.5m from marker
        self.declare_parameter('search_angular_speed', 0.4)   # rad/s when searching
        self.declare_parameter('approach_linear_speed', 0.25) # m/s when approaching
        self.declare_parameter('approach_angular_speed', 0.3) # rad/s for centering
        self.declare_parameter('loop', True)                  # Restart after last marker
        self.declare_parameter('pause_duration', 2.0)         # Pause at each marker (seconds)
        self.declare_parameter('camera_frame', 'camera_rgb_frame')
        self.declare_parameter('centering_threshold', 0.1)    # Marker center tolerance (m)

        # Get parameters
        self.marker_sequence = self.get_parameter('marker_sequence').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.search_angular_speed = self.get_parameter('search_angular_speed').value
        self.approach_linear_speed = self.get_parameter('approach_linear_speed').value
        self.approach_angular_speed = self.get_parameter('approach_angular_speed').value
        self.loop = self.get_parameter('loop').value
        self.pause_duration = self.get_parameter('pause_duration').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.centering_threshold = self.get_parameter('centering_threshold').value

        # State machine
        self.state = PatrolState.SEARCHING
        self.current_marker_index = 0
        self.pause_start_time = None
        self.marker_lost_count = 0
        self.max_lost_count = 10  # Number of cycles before declaring marker lost

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/aruco/patrol_status',
            10
        )

        # Main control loop timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Patrol Navigator initialized')
        self.get_logger().info(f'  Marker sequence: {self.marker_sequence}')
        self.get_logger().info(f'  Approach distance: {self.approach_distance}m')

    @property
    def current_target_marker_id(self):
        """Get the ID of the current target marker."""
        if self.current_marker_index < len(self.marker_sequence):
            return self.marker_sequence[self.current_marker_index]
        return None

    def get_marker_transform(self, marker_id):
        """
        Get the transform from camera to marker.
        Returns (x, y, z) position or None if not found.
        """
        try:
            # Look up transform from camera to marker
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                f'aruco_marker_{marker_id}',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            t = transform.transform.translation
            return (t.x, t.y, t.z)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def control_loop(self):
        """Main control loop - state machine execution."""
        cmd = Twist()
        status_msg = String()

        target_id = self.current_target_marker_id

        if self.state == PatrolState.SEARCHING:
            status_msg.data = f'SEARCHING for marker {target_id}'
            
            # Try to find the target marker
            marker_pos = self.get_marker_transform(target_id)
            
            if marker_pos is not None:
                # Found marker! Transition to APPROACH
                self.state = PatrolState.APPROACH
                self.marker_lost_count = 0
                self.get_logger().info(f'Found marker {target_id}, approaching...')
            else:
                # Rotate to search for marker
                cmd.angular.z = self.search_angular_speed

        elif self.state == PatrolState.APPROACH:
            marker_pos = self.get_marker_transform(target_id)
            
            if marker_pos is None:
                # Lost marker
                self.marker_lost_count += 1
                if self.marker_lost_count > self.max_lost_count:
                    self.state = PatrolState.SEARCHING
                    self.get_logger().warn(f'Lost marker {target_id}, searching...')
                else:
                    # Keep last velocity briefly
                    pass
            else:
                self.marker_lost_count = 0
                x, y, z = marker_pos
                
                # z is forward distance, x is left/right offset
                distance = z
                lateral_offset = x
                
                status_msg.data = f'APPROACH marker {target_id} | dist: {distance:.2f}m | offset: {lateral_offset:.2f}m'
                
                # Check if we've reached the marker
                if distance <= self.approach_distance:
                    self.state = PatrolState.NEXT_MARKER
                    self.pause_start_time = self.get_clock().now()
                    self.get_logger().info(f'Reached marker {target_id}!')
                else:
                    # Drive toward marker
                    # Angular: correct lateral offset (turn toward marker)
                    if abs(lateral_offset) > self.centering_threshold:
                        # Negative x means marker is to the right, turn right (negative angular)
                        cmd.angular.z = -self.approach_angular_speed * (lateral_offset / abs(lateral_offset))
                    
                    # Linear: drive forward (proportional to distance, with min/max)
                    forward_speed = min(self.approach_linear_speed, 
                                       self.approach_linear_speed * (distance - self.approach_distance))
                    forward_speed = max(0.05, forward_speed)  # Minimum speed
                    cmd.linear.x = forward_speed

        elif self.state == PatrolState.NEXT_MARKER:
            status_msg.data = f'PAUSING at marker {target_id}'
            
            # Pause at marker
            elapsed = (self.get_clock().now() - self.pause_start_time).nanoseconds / 1e9
            
            if elapsed >= self.pause_duration:
                # Move to next marker
                self.current_marker_index += 1
                
                if self.current_marker_index >= len(self.marker_sequence):
                    if self.loop:
                        self.current_marker_index = 0
                        self.state = PatrolState.SEARCHING
                        self.get_logger().info('Patrol complete! Looping back to start...')
                    else:
                        self.state = PatrolState.COMPLETE
                        self.get_logger().info('Patrol complete!')
                else:
                    self.state = PatrolState.SEARCHING
                    self.get_logger().info(f'Moving to next marker: {self.current_target_marker_id}')

        elif self.state == PatrolState.COMPLETE:
            status_msg.data = 'COMPLETE - All markers visited'
            # Stop the robot
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)
        
        # Publish status
        self.status_pub.publish(status_msg)

    def stop_robot(self):
        """Stop the robot immediately."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
