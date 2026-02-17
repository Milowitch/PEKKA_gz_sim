import sys
import rclpy
from rover_interfaces.srv import ManageWaypoint  # ใช้ service ที่เราเตรียมไว้

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('waypoint_client')

    if len(sys.argv) < 3:
        print("Usage: waypoint_client <command> <name> [x y]")
        print("Commands:")
        print("  set <name> <x> <y>   - Set a waypoint with a given name and coordinates")
        print("  delete <name>        - Delete a waypoint by its name")
        print("  get <name>           - Get a waypoint by its name")
        print("  go_to <name>         - Command the robot to go to a waypoint")
        return

    command = sys.argv[1]
    name = sys.argv[2]

    # ตรวจสอบว่า command ที่ได้รับถูกต้องหรือไม่
    if command not in ['set', 'delete', 'get', 'go_to']:
        print(f"Error: Unknown command '{command}'")
        print("Valid commands are: set, delete, get, go_to")
        return

    client = node.create_client(ManageWaypoint, 'manage_waypoint')

    # รอให้ service พร้อมใช้งาน
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')

    # สร้าง request ที่จะส่งไปยัง service
    request = ManageWaypoint.Request()
    request.command = command
    request.name = name

    # สำหรับคำสั่ง 'set', ต้องมีพิกัด x และ y
    if command == 'set' and len(sys.argv) == 5:
        try:
            request.x = float(sys.argv[3])
            request.y = float(sys.argv[4])
        except ValueError:
            print("Error: x and y should be numeric values.")
            return

    # สำหรับคำสั่ง 'go_to', ไม่มีพิกัด แต่ใช้เพียงชื่อ waypoint
    if command == 'go_to':
        pass  # ไม่มีการส่งพิกัด, ใช้เพียงชื่อ waypoint

    # เรียกใช้ service และรอผล
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f'Service response: {future.result().message}')
    else:
        node.get_logger().error('Service call failed.')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
