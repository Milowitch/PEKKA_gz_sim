#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import numpy as np
import subprocess
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Int16

# Define the list of preferred adapters in order
preferred_adapters = [
    'wlx202351dd8642',  # TP-Link Micro B
    'wlx081f7125abd0',  # TP-Link USB
    'wlo1',             # Built-in adapter
    'wlp4s0',           # Built-in adapter
    'wlp111s0f0',       # Built-in adapter
]

ssid_options = ['TP-rover', 'TP-rover 1', 'TP-rover 2', 'TP-rover 3']  # Replace with your network SSID
password = "Sl33plai3"  # Replace with your network password

class Robot_Health(Node):

    def __init__(self):
        super().__init__('robot_health')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        self.declare_parameter('preferred_adapters', preferred_adapters)
        self.declare_parameter('ssid_options', ssid_options)
        self.declare_parameter('password', password)
        self.declare_parameter('wifi_auto_connect', False)  # Declare the new parameter

        self.temp_limit_subscription = self.create_subscription(
            Float64,
            '/temp_limit',
            self.temp_limit_callback,
            qos_profile,
        )
        
        self.temp_limit = 25.0 # ํC

        self.wifi_status = 0
        self.pub_health_command = self.create_publisher(Int16, '/wifi_status', qos_profile)
        self._timer = self.create_timer(5.0, self.publish)  # Adjust timer interval as needed
    
    def temp_limit_callback(self, temp_limit_msg):
        self.temp_limit = temp_limit_msg.data

    def get_wifi_devices(self):
        try:
            result = subprocess.run(['nmcli', '-t', '-f', 'DEVICE', 'device'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            output = result.stdout.decode('utf-8').strip()
            return output.split('\n')
        except Exception as e:
            self.get_logger().error(f"Error getting Wi-Fi devices: {str(e)}")
            return []

    def check_wifi_adapter_status(self, adapter_name):
        try:
            result = subprocess.run(['nmcli', '-t', '-f', 'DEVICE,TYPE,STATE', 'device'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            output = result.stdout.decode('utf-8').strip()

            for line in output.split('\n'):
                device_info = line.split(':')
                if len(device_info) == 3 and device_info[0] == adapter_name and device_info[1] == 'wifi':
                    return device_info[2]
            return "unknown"
        except Exception as e:
            self.get_logger().error(f"Error checking WiFi adapter status: {str(e)}")
            return "unknown"

    def get_connected_ssid(self, adapter_name):
        try:
            result = subprocess.run(['nmcli', '-t', '-f', 'ACTIVE,DEVICE,NAME', 'connection', 'show', '--active'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            output = result.stdout.decode('utf-8').strip()

            for line in output.split('\n'):
                fields = line.split(':')
                if len(fields) == 3 and fields[0] == 'yes' and fields[1] == adapter_name:
                    return fields[2]

            return None

        except Exception as e:
            self.get_logger().error(f"Error checking connected SSID: {str(e)}")
            return None

    def auto_connect_wifi(self, ssid_options, password):
        # Only attempt auto-connect if wifi_auto_connect is True
        if not self.get_parameter('wifi_auto_connect').value:
            self.get_logger().info("Wi-Fi auto-connect is disabled.")
            return

        wifi_devices = self.get_wifi_devices()
        if len(wifi_devices) == 0:
            self.get_logger().info("No Wi-Fi devices found.")
            return

        # Check if any adapter is already connected to any of the acceptable SSIDs
        connected_adapter = None
        for adapter in self.get_parameter('preferred_adapters').value:
            if adapter in wifi_devices:
                adapter_status = self.check_wifi_adapter_status(adapter)
                connected_ssid = self.get_connected_ssid(adapter)

                if "connected" in adapter_status and connected_ssid in ssid_options:
                    connected_adapter = adapter
                    break

        if connected_adapter:
            self.get_logger().info(f"Adapter '{connected_adapter}' is already connected to one of the acceptable SSIDs.")
            return  # No need to reconnect if an acceptable SSID is already connected

        # If no adapter is connected to the acceptable SSIDs, attempt to connect using the preferred adapters
        for adapter in self.get_parameter('preferred_adapters').value:
            if adapter in wifi_devices:
                adapter_status = self.check_wifi_adapter_status(adapter)
                if "connected" in adapter_status:
                    connected_ssid = self.get_connected_ssid(adapter)
                    if connected_ssid not in ssid_options:
                        self.get_logger().info(f"Adapter '{adapter}' is connected to a different SSID '{connected_ssid}'. Disconnecting and reconnecting...")
                        self.disconnect_from_network(adapter)
                        self.connect_to_network(adapter, ssid_options[0], password)  # Connect to the first acceptable SSID
                        return
                else:
                    self.get_logger().info(f"Attempting to connect using {adapter}")
                    self.connect_to_network(adapter, ssid_options[0], password)  # Connect to the first acceptable SSID
                    return  # Stop after connecting

        self.get_logger().info("No preferred adapters are available. Please check the devices.")


    def disconnect_from_network(self, adapter_name):
        try:
            result = subprocess.run(['nmcli', 'device', 'disconnect', adapter_name], capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info(f"Successfully disconnected from network on {adapter_name}")
            else:
                self.get_logger().error(f"Failed to disconnect from network on {adapter_name}: {result.stderr}")
        except Exception as e:
            self.get_logger().error(f"Error disconnecting from network: {str(e)}")

    def connect_to_network(self, adapter_name, ssid, password):
        try:
            result = subprocess.run(['nmcli', 'device', 'wifi', 'connect', ssid, 'password', password, 'ifname', adapter_name], capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info(f"Successfully connected to '{ssid}' using {adapter_name}")
            else:
                self.get_logger().error(f"Failed to connect to '{ssid}' using {adapter_name}: {result.stderr}")
        except Exception as e:
            self.get_logger().error(f"Error connecting to network: {str(e)}")

    def publish(self):
        wifi_devices = self.get_wifi_devices()
        adapter_found = False

        for adapter_name in self.get_parameter('preferred_adapters').value:
            if adapter_name in wifi_devices:
                self.auto_connect_wifi(self.get_parameter('ssid_options').value, self.get_parameter('password').value)

                adapter_status = self.check_wifi_adapter_status(adapter_name)

                if adapter_status == 'unavailable':
                    self.wifi_status = 0 # RED
                elif adapter_status == 'disconnected':
                    self.wifi_status = 1 # YELLOW
                elif adapter_status == 'connected':
                    self.wifi_status = 2 # GREEN
                else:
                    self.wifi_status = 3 # OFF

                adapter_found = True
                break

        if not adapter_found:
            self.wifi_status = 0 # RED

        wifi_status = Int16()
        wifi_status.data = self.wifi_status
        self.pub_health_command.publish(wifi_status)

def main(args=None):
    rclpy.init(args=args)

    robot_health = Robot_Health()

    rclpy.spin(robot_health)

    # Destroy the node explicitly
    robot_health.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
