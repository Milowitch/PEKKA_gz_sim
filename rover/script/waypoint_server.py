import rclpy
from rclpy.node import Node
from rover_interfaces.srv import ManageWaypoint  # ใช้ service ที่มาจาก rover_interfaces
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class WaypointServer(Node):
    def __init__(self):
        super().__init__('waypoint_server')
        self.srv = self.create_service(ManageWaypoint, 'manage_waypoint', self.manage_waypoint_callback)
        self.get_logger().info("Waypoint server is ready.")
        self.waypoints = {}  # ใช้สำหรับเก็บข้อมูลของ waypoint
        self._navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def manage_waypoint_callback(self, request, response):
        command = request.command
        name = request.name
        self.get_logger().info(f"Received command: {command} for waypoint: {name}")

        if command == 'set':
            # Logic สำหรับตั้ง waypoint
            self.waypoints[name] = (request.x, request.y)
            response.success = True
            response.message = f"Waypoint {name} set at {request.x}, {request.y}"
        elif command == 'delete':
            # Logic สำหรับลบ waypoint
            if name in self.waypoints:
                del self.waypoints[name]
                response.success = True
                response.message = f"Waypoint {name} deleted."
            else:
                response.success = False
                response.message = f"Waypoint {name} not found."
        elif command == 'get':
            # Logic สำหรับดึงข้อมูล waypoint
            if name in self.waypoints:
                x, y = self.waypoints[name]
                response.success = True
                response.message = f"Waypoint {name} info: ({x}, {y})"
            else:
                response.success = False
                response.message = f"Waypoint {name} not found."
        elif command == 'go_to':
            # Logic สำหรับให้หุ่นยนต์ไปที่ waypoint (2D Goal Pose)
            if name in self.waypoints:
                # ส่งคำสั่งไปที่ Nav2
                x, y = self.waypoints[name]
                goal = PoseStamped()
                goal.header.frame_id = 'map'  # หรือ frame_id ที่หุ่นยนต์ใช้
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.orientation.w = 1.0  # ตั้ง orientation เป็น 1 (ไม่มีการหมุน)

                # ส่ง goal ไปยัง Nav2
                self.send_goal_to_nav2(goal)
                response.success = True
                response.message = f"Going to waypoint {name} at ({x}, {y})"
            else:
                response.success = False
                response.message = f"Waypoint {name} not found."
        else:
            response.success = False
            response.message = "Unknown command."

        return response

    def send_goal_to_nav2(self, goal: PoseStamped):
        # รอให้ action client เชื่อมต่อกับ Nav2
        if not self._navigate_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 server not available!')
            return
        
        # สร้างคำขอ goal ให้ Nav2
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        # ส่ง goal ไปยัง Nav2
        self._navigate_to_pose_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    waypoint_server = WaypointServer()
    rclpy.spin(waypoint_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()