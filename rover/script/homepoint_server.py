import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import NavigateToPose
from std_srvs.srv import Trigger
import math

class HomeService(Node):
    def __init__(self):
        super().__init__('home_service')

        self.home_pose = None  # เก็บตำแหน่ง home point

        # Subscriber สำหรับอ่าน odometry
        self.odom_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.odom_callback,
            10)

        # สร้าง Service
        self.set_home_srv = self.create_service(Trigger, 'set_home', self.set_home_callback)
        self.remove_home_srv = self.create_service(Trigger, 'remove_home', self.remove_home_callback)
        self.get_home_srv = self.create_service(Trigger, 'get_home', self.get_home_callback)
        self.go_home_srv = self.create_service(Trigger, 'go_home', self.go_home_callback)

        # Client สำหรับ Nav2
        self.nav_client = self.create_client(NavigateToPose, '/navigate_to_pose')

    def odom_callback(self, msg):
        """ Callback จาก /amcl_pose """
        self.current_pose = msg.pose.pose

    def set_home_callback(self, request, response):
        """ ตั้งค่าจุด home """
        if hasattr(self, 'current_pose'):
            self.home_pose = self.current_pose
            self.get_logger().info("✅ Home set at x={}, y={}, theta={}".format(
                self.home_pose.position.x, 
                self.home_pose.position.y, 
                math.atan2(
                    self.home_pose.orientation.z, 
                    self.home_pose.orientation.w
                )
            ))
            response.success = True
            response.message = "Home point set successfully"
        else:
            response.success = False
            response.message = "Failed to set home"
        return response

    def remove_home_callback(self, request, response):
        """ ลบ home point """
        self.home_pose = None
        self.get_logger().info("❌ Home removed")
        response.success = True
        response.message = "Home point removed"
        return response

    def get_home_callback(self, request, response):
        """ ส่งค่าจุด home """
        if self.home_pose:
            response.success = True
            response.message = "Home position: x={}, y={}".format(
                self.home_pose.position.x, self.home_pose.position.y)
        else:
            response.success = False
            response.message = "No home point set"
        return response

    def go_home_callback(self, request, response):
        """ ส่งคำสั่งให้ Nav2 ไป home point """
        if not self.home_pose:
            self.get_logger().error("🚫 No home point set")
            response.success = False
            response.message = "No home point set"
            return response

        goal_msg = NavigateToPose.Request()
        goal_msg.pose.pose = self.home_pose
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        future = self.nav_client.call_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        response.success = True if future.result() else False
        response.message = "Navigating to home"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = HomeService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
