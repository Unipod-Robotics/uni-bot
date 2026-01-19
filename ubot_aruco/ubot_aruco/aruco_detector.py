#!/usr/bin/env python3
"""
ArUco Marker Detector Node

Detects ArUco markers from camera images and publishes:
- Marker poses as TF transforms
- Visualization markers for RViz
- Detection messages with marker IDs and poses
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

from tf2_ros import TransformBroadcaster


class ArucoDetector(Node):
    """ROS 2 node for detecting ArUco markers."""

    def __init__(self):
        super().__init__('aruco_detector')

        # Declare parameters
        self.declare_parameter('marker_size', 0.15)  # 15cm markers
        self.declare_parameter('dictionary_id', 0)   # DICT_4X4_50
        self.declare_parameter('camera_frame', 'camera_rgb_frame')
        self.declare_parameter('image_topic', '/camera/rgb/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/rgb/camera_info')

        # Get parameters
        self.marker_size = self.get_parameter('marker_size').value
        dict_id = self.get_parameter('dictionary_id').value
        self.camera_frame = self.get_parameter('camera_frame').value
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value

        # Initialize ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Camera intrinsics (will be updated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            sensor_qos
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            sensor_qos
        )

        # Publishers
        self.markers_pub = self.create_publisher(
            MarkerArray,
            '/aruco/visualization_markers',
            10
        )

        self.detections_pub = self.create_publisher(
            PoseStamped,
            '/aruco/detected_marker',
            10
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            '/aruco/debug_image',
            10
        )

        self.get_logger().info(f'ArUco Detector initialized')
        self.get_logger().info(f'  Marker size: {self.marker_size}m')
        self.get_logger().info(f'  Listening on: {image_topic}')

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics from camera_info topic."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received')

    def image_callback(self, msg: Image):
        """Process incoming camera images for ArUco markers."""
        if self.camera_matrix is None:
            return  # Wait for camera info

        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(cv_image)

        # Prepare visualization
        marker_array = MarkerArray()
        debug_image = cv_image.copy()

        if ids is not None and len(ids) > 0:
            # Draw detected markers on debug image
            cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)

            # Estimate pose for each marker
            for i, marker_id in enumerate(ids.flatten()):
                # Get marker corners
                marker_corners = corners[i]

                # Estimate pose using solvePnP
                obj_points = np.array([
                    [-self.marker_size/2,  self.marker_size/2, 0],
                    [ self.marker_size/2,  self.marker_size/2, 0],
                    [ self.marker_size/2, -self.marker_size/2, 0],
                    [-self.marker_size/2, -self.marker_size/2, 0]
                ], dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    marker_corners.reshape(-1, 2),
                    self.camera_matrix,
                    self.dist_coeffs
                )

                if success:
                    # Draw axis on debug image
                    cv2.drawFrameAxes(
                        debug_image,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec, tvec,
                        self.marker_size * 0.5
                    )

                    # Convert rotation vector to rotation matrix
                    rotation_matrix, _ = cv2.Rodrigues(rvec)

                    # Convert to quaternion
                    quat = self.rotation_matrix_to_quaternion(rotation_matrix)

                    # Publish TF transform
                    self.publish_tf(marker_id, tvec.flatten(), quat, msg.header.stamp)

                    # Publish detection message
                    self.publish_detection(marker_id, tvec.flatten(), quat, msg.header.stamp)

                    # Add visualization marker
                    viz_marker = self.create_viz_marker(marker_id, tvec.flatten(), quat, msg.header.stamp)
                    marker_array.markers.append(viz_marker)

        # Publish visualization markers
        self.markers_pub.publish(marker_array)

        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Debug image publish error: {e}')

    def rotation_matrix_to_quaternion(self, R):
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return [x, y, z, w]

    def publish_tf(self, marker_id, translation, quaternion, stamp):
        """Publish TF transform for detected marker."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.camera_frame
        t.child_frame_id = f'aruco_marker_{marker_id}'

        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])

        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])

        self.tf_broadcaster.sendTransform(t)

    def publish_detection(self, marker_id, translation, quaternion, stamp):
        """Publish detected marker as PoseStamped."""
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = f'aruco_marker_{marker_id}'

        pose.pose.position.x = float(translation[0])
        pose.pose.position.y = float(translation[1])
        pose.pose.position.z = float(translation[2])

        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])

        self.detections_pub.publish(pose)

    def create_viz_marker(self, marker_id, translation, quaternion, stamp):
        """Create RViz visualization marker."""
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.camera_frame
        marker.ns = 'aruco_markers'
        marker.id = int(marker_id)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = float(translation[0])
        marker.pose.position.y = float(translation[1])
        marker.pose.position.z = float(translation[2])

        marker.pose.orientation.x = float(quaternion[0])
        marker.pose.orientation.y = float(quaternion[1])
        marker.pose.orientation.z = float(quaternion[2])
        marker.pose.orientation.w = float(quaternion[3])

        marker.scale.x = self.marker_size
        marker.scale.y = self.marker_size
        marker.scale.z = 0.01

        # Color based on marker ID
        colors = [
            (1.0, 0.0, 0.0),  # Red
            (0.0, 1.0, 0.0),  # Green
            (0.0, 0.0, 1.0),  # Blue
            (1.0, 1.0, 0.0),  # Yellow
            (1.0, 0.0, 1.0),  # Magenta
        ]
        color = colors[marker_id % len(colors)]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.8

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 500000000  # 0.5 seconds

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
