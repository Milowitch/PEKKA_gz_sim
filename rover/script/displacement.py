import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class WaypointListener(Node):
    def __init__(self):
        super().__init__('waypoint_listener')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # หัวข้อที่ RViz2 ส่ง Waypoints
            self.waypoint_callback,
            10)
        self.current_waypoint = None
        self.current_pose = None

        # สมัครสมาชิกหัวข้อตำแหน่งปัจจุบันของหุ่นยนต์
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/odom',  # ขึ้นอยู่กับการตั้งค่าของคุณ
            self.pose_callback,
            10)

    def waypoint_callback(self, msg):
        self.current_waypoint = msg
        self.get_logger().info(f'Received new waypoint: {msg.pose.position}')
        self.calculate_and_log_displacement()

    def pose_callback(self, msg):
        self.current_pose = msg
        self.calculate_and_log_displacement()

    def calculate_and_log_displacement(self):
        if self.current_pose and self.current_waypoint:
            displacement = self.calculate_displacement(
                self.current_pose.pose, self.current_waypoint.pose)
            self.get_logger().info(f'Displacement to waypoint: {displacement} meters')

    @staticmethod
    def calculate_displacement(current_pose, waypoint_pose):
        dx = waypoint_pose.position.x - current_pose.position.x
        dy = waypoint_pose.position.y - current_pose.position.y
        displacement = math.sqrt(dx**2 + dy**2)
        return displacement

def main(args=None):
    rclpy.init(args=args)
    node = WaypointListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
