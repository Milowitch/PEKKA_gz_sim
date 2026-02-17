import os
import subprocess
import rclpy
from rclpy.node import Node

class CameraSymlinkNode(Node):
    def __init__(self):
        super().__init__('camera_symlink')
        self.target_vendor_id = "0ac8"  # Example vendor ID
        self.target_product_id = "0345"  # Example product ID
        self.target_index = "0"  # Example index
        self.video4linux_path = "/dev/"
        self.device_count = 0
        self.create_camera_symlinks()

    def get_device_info(self, device_path):
        try:
            result = subprocess.run(
                ['udevadm', 'info', '--attribute-walk', '--name', device_path],
                capture_output=True, text=True, check=True
            )
            return result.stdout
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error getting device info for {device_path}: {e}")
            return ""

    def create_symlink(self, device_path):
        symlink_name = f"/dev/cam{self.device_count}"
        device_name = os.path.basename(device_path)

        if os.path.exists(symlink_name) or os.path.islink(symlink_name):
            os.remove(symlink_name)

        try:
            os.symlink(device_name, symlink_name)
            self.get_logger().info(f"Created symlink: {symlink_name} -> {device_name}")
            self.device_count += 1
        except PermissionError:
            self.get_logger().error(f"Permission denied while creating symlink for /dev/{device_name}")

    def create_camera_symlinks(self):
        devices = [os.path.join(self.video4linux_path, device) for device in os.listdir(self.video4linux_path) if device.startswith("video")]

        for device_path in devices:
            device_info = self.get_device_info(device_path)
            if self.target_vendor_id in device_info and self.target_product_id in device_info:
                index_line = [line for line in device_info.splitlines() if "ATTR{index}" in line]
                if index_line and f'ATTR{{index}}=="{self.target_index}"' in index_line[0]:
                    self.create_symlink(device_path)
        
        self.get_logger().info(f"Found {self.device_count} devices matching the criteria.")


def main(args=None):
    rclpy.init(args=args)
    node = CameraSymlinkNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()