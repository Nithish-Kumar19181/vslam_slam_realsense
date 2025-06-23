#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class DepthToLaserScanNode(Node):
    def __init__(self):
        super().__init__('depth_to_laserscan')

        # Camera intrinsics for RealSense D455 (adjust if needed)
        self.fx = 617.0
        self.fy = 617.0
        self.cx = 320.0
        self.cy = 240.0

        # LaserScan parameters
        self.angle_min = -1.57   # -90 degrees
        self.angle_max = 1.57    # +90 degrees
        self.range_min = 0.3     # meters
        self.range_max = 5.0     # meters

        self.depth_topic = '/camera/camera/depth/image_rect_raw'
        self.scan_topic = '/scan'

        self.bridge = CvBridge()
        self.latest_msg = None

        self.sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(LaserScan, self.scan_topic, 10)

        # Publish at 10 Hz
        self.publish_timer = self.create_timer(0.5, self.publish_scan)

    def depth_callback(self, msg):
        self.latest_msg = msg

    def publish_scan(self):
        if self.latest_msg is None:
            return

        msg = self.latest_msg

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        if len(cv_image.shape) != 2:
            self.get_logger().error('Depth image is not 2D!')
            return

        height, width = cv_image.shape
        center_row_idx = height // 2
        band_height = 1
        rows = np.arange(center_row_idx - band_height // 2, center_row_idx + band_height // 2 + 1)

        depths = cv_image[rows, :].astype(np.float32) / 1000.0  # convert mm to meters
        depth_row = np.mean(depths, axis=0)

        u_coords = np.arange(width)
        v = center_row_idx

        X = (u_coords - self.cx) * depth_row / self.fx
        Z = depth_row

        angles = np.arctan2(X, Z)

        valid_angle = np.logical_and(angles >= self.angle_min, angles <= self.angle_max)
        valid_range = np.logical_and(depth_row > self.range_min, depth_row < self.range_max)
        valid = np.logical_and(valid_angle, valid_range)

        ranges = np.full(width, float('inf'))
        ranges[valid] = np.sqrt(X[valid]**2 + Z[valid]**2)

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp  # Use the same timestamp as the image
        scan.header.frame_id = "base_link"    # Ensure TF tree supports this frame

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = (self.angle_max - self.angle_min) / (width - 1)  # gives 641 points
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()

        self.pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = DepthToLaserScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
