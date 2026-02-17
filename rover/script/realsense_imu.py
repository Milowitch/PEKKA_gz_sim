import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

class IMURawPublisher(Node):
    def __init__(self):
        super().__init__('imu_raw_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber รับข้อมูลจาก IMU
        self.imu_sub = self.create_subscription(Imu, '/rover/D456_1/imu', self.imu_callback, qos_profile)

        # Publisher ส่งข้อมูลไปยัง /rover/D456_1/imu/rawdata
        self.imu_pub = self.create_publisher(Imu, '/rover/D456_1/imu/rawdata', qos_profile)

        # ตัวแปรเก็บมุมสะสม
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # ตัวแปรเก็บเวลา
        self.last_time = self.get_clock().now().to_msg()

    def imu_callback(self, msg):
        current_time = self.get_clock().now().to_msg()
        dt = (current_time.sec - self.last_time.sec) + (current_time.nanosec - self.last_time.nanosec) / 1e9
        self.last_time = current_time

        # คำนวณการเปลี่ยนแปลงของมุมจาก angular velocity
        self.roll += msg.angular_velocity.z * dt
        self.pitch += msg.angular_velocity.x * dt
        self.yaw += msg.angular_velocity.y * dt

        # แปลง Euler angles เป็น Quaternion
        quat = quaternion_from_euler(self.roll, -self.pitch, -self.yaw)

        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = msg.header.frame_id  # ใช้ frame เดิม

        # ตั้งค่า orientation ที่คำนวณได้
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

        # ตั้งค่า covariance (สมมุติค่าต่ำ เพราะไม่มีการคำนวณฟิลเตอร์ขั้นสูง)
        imu_msg.orientation_covariance = [0.01] * 9  

        # คัดลอก angular velocity และ linear acceleration
        imu_msg.angular_velocity = msg.angular_velocity
        imu_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        imu_msg.linear_acceleration = msg.linear_acceleration
        imu_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.imu_pub.publish(imu_msg)
        # self.get_logger().info('Published IMU raw data with estimated orientation')

def main():
    rclpy.init()
    node = IMURawPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
