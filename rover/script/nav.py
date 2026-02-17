#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_transformations
import math
from datetime import datetime, timedelta

from numpy import interp

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        self.p1 = [0.0, 0.0, 0.0] # linear_acceleration.x      linear_acceleration.y      linear_acceleration.z
        self.p2 = [0.0, 0.0, 0.0] # angular_velocity.x         angular_velocity.y         angular_velocity.z
        self.p3 = [0.0, 0.0, 0.0] # orientation.x              orientation.y              orientation.z
        self.p4 = [0.0, 0.0, 0.0] # magnetic_field.x           magnetic_field.y           magnetic_field.z
        self.p5 = [0.0, 0.0, 0.0] # latitude                   longitude                  altitude
        self.p6 = [0.0, 0.0, 0.0] # satellites                 date                       time

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.nav_subscription = self.create_subscription(
            Float64MultiArray,
            '/nav/rawdata',
            self.nav_callback,
            qos_profile,
        )
        self.nav_subscription
        self.publisher_imu = self.create_publisher(Imu, '/imu/rawdata', qos_profile)
        self.publisher_mag = self.create_publisher(MagneticField, '/mag/rawdata', qos_profile)
        self.publisher_gps = self.create_publisher(NavSatFix, '/gps/rawdata', qos_profile)
        self.publisher_gps_date = self.create_publisher(String, '/gps/date', qos_profile)
        self.publisher_gps_time = self.create_publisher(String, '/gps/time', qos_profile)
        self._timer = self.create_timer(0.1, self.publish_my_data_vel)

    def nav_callback(self, nav_msg):
        self.p1 = [nav_msg.data[0], nav_msg.data[1], nav_msg.data[2]]
        self.p2 = [nav_msg.data[3], nav_msg.data[4], nav_msg.data[5]]
        self.p3 = [nav_msg.data[6], nav_msg.data[7], nav_msg.data[8]]
        self.p4 = [nav_msg.data[9], nav_msg.data[10], nav_msg.data[11]]
        self.p5 = [nav_msg.data[12], nav_msg.data[13], nav_msg.data[14]]
        self.p6 = [nav_msg.data[15], nav_msg.data[16], nav_msg.data[17]]

    def publish_my_data_vel(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"  # You can change the frame ID as needed
        # Fill linear acceleration from p1
        imu_msg.linear_acceleration = Vector3(x=self.p1[0], y=self.p1[1], z=self.p1[2])
        # Fill angular velocity from p2
        imu_msg.angular_velocity = Vector3(x=self.p2[0], y=self.p2[1], z=self.p2[2])
        # Fill orientation from p3 (as quaternion, assuming x, y, z are used in the quaternion)
        # Convert angles from degrees to radians
        pitch = math.radians(-self.p3[0])
        roll = math.radians(self.p3[1])
        yaw = math.radians(self.p3[2])
        # Convert Euler angles to quaternion
        quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        # Assign the quaternion to imu_msg.orientation
        imu_msg.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        self.publisher_imu.publish(imu_msg)

        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = "mag_link"  # You can change the frame ID as needed
        # Fill magnetic field from p4
        mag_msg.magnetic_field = Vector3(x=self.p4[0], y=self.p4[1], z=self.p4[2])
        self.publisher_mag.publish(mag_msg)

        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_link"  # You can change the frame ID as needed
        # Fill GPS data from p5
        gps_msg.latitude = self.p5[0]
        gps_msg.longitude = self.p5[1]
        gps_msg.altitude = self.p5[2]
        self.publisher_gps.publish(gps_msg)

        gps_date_msg = String()
        try:
            gps_date_msg.data = str(int(self.p6[1]))
        except:
            pass
        self.publisher_gps_date.publish(gps_date_msg)
        gps_time_msg = String()
        try:
            raw_time_str = str(int(self.p6[2]))  # แปลงเป็น int เพื่อตัด .0 ออกก่อน
            hmmss = raw_time_str[:-2]  # เอาเฉพาะ HMMSS
            hhmmss = hmmss.zfill(6)  # เติม 0 จนกว่าจะเป็น 6 ตัว
            utc_time = datetime.strptime(hhmmss, "%H%M%S")
            bangkok_time = utc_time + timedelta(hours=7)
            gps_time_msg.data = bangkok_time.strftime("%H%M%S")  # แปลงกลับเป็น HHMMSS
        except Exception as e:
            print(f"Error: {e}")  # Print error message
            pass
        self.publisher_gps_time.publish(gps_time_msg)

        # transform = TransformStamped()
        # transform.header.stamp = self.get_clock().now().to_msg()
        # transform.header.frame_id = 'base_frame'
        # transform.child_frame_id = 'base_link'
        # transform.transform.rotation = imu_msg.orientation
        # # Broadcast the transform
        # self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()