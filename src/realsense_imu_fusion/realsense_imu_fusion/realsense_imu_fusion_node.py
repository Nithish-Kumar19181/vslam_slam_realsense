#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

class RealsenseIMUFusion(Node):
    def __init__(self):
        super().__init__('realsense_imu_fusion_node')

        self.get_logger().info('✔️ RealSense IMU Fusion Node Started')

        self.accel_msg = None
        self.gyro_msg = None

        # ✅ Publishes fused IMU to /camera/imu
        self.imu_pub = self.create_publisher(
            Imu,
            '/camera/imu',
            qos_profile_sensor_data
        )

        # ✅ Subscribe to RealSense raw accelerometer and gyroscope data
        self.accel_sub = self.create_subscription(
            Imu,
            '/camera/camera/accel/sample',
            self.accel_callback,
            qos_profile_sensor_data
        )

        self.gyro_sub = self.create_subscription(
            Imu,
            '/camera/camera/gyro/sample',
            self.gyro_callback,
            qos_profile_sensor_data
        )

    def accel_callback(self, msg):
        self.accel_msg = msg
        self.try_publish()

    def gyro_callback(self, msg):
        self.gyro_msg = msg
        self.try_publish()

    def try_publish(self):
        # Only publish when both accel and gyro data are available
        if self.accel_msg is None or self.gyro_msg is None:
            return

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'camera_link'  # Must match your TF tree

        # Fill in angular velocity and linear acceleration
        imu_msg.angular_velocity = self.gyro_msg.angular_velocity
        imu_msg.angular_velocity_covariance = self.gyro_msg.angular_velocity_covariance

        imu_msg.linear_acceleration = self.accel_msg.linear_acceleration
        imu_msg.linear_acceleration_covariance = self.accel_msg.linear_acceleration_covariance

        imu_msg.orientation_covariance[0] = -1.0  # Indicates orientation is unknown

        self.imu_pub.publish(imu_msg)

        # Reset messages after publishing
        self.accel_msg = None
        self.gyro_msg = None

def main(args=None):
    rclpy.init(args=args)
    node = RealsenseIMUFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
