#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import os
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

from numpy import interp

class Station(Node):

    def __init__(self):
        super().__init__('station')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        self.declare_parameter('js', 0)
        self.publisher_ = self.create_publisher(String, '/station_joytype', qos_profile)
        self._timer = self.create_timer(0.5, self.publish_joytype)

    def publish_joytype(self):
        joytype = String()
        joytype.data = self.get_joystick_connection_type(self.get_parameter('js').value)
        self.publisher_.publish(joytype)
    
    def get_joystick_connection_type(self, joystick_id=0):
        # Path to the joystick device
        device_path = f"/sys/class/input/js{joystick_id}/device"
        
        if not os.path.exists(device_path):
            print(f"Joystick {joystick_id} not found.")
            return "Not Connection"

        # Check the parent path to identify the connection type
        parent_path = os.path.realpath(device_path + "/..")
        
        # print(f"Parent path for joystick {joystick_id}: {parent_path}")  # Debugging line
        
        # Identify if it's a USB (wired) or Bluetooth connection
        if "usb" in parent_path.lower():
            return "Wired (USB)"
        elif "bluetooth" in parent_path.lower() or "bt" in parent_path.lower():
            return "Bluetooth"
        elif "uhid" in parent_path.lower():
            return "Bluetooth (UHID)"
        else:
            print(f"Could not identify connection type for path: {parent_path}")  # Debugging line
            return "Unknown"

def main(args=None):
    rclpy.init(args=args)

    station = Station()

    rclpy.spin(station)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    station.destroy_node()
    rclpy.shutdown()

def mymap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if __name__ == '__main__':
    main()