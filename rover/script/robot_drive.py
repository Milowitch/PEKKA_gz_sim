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

class Robot_Drive(Node):

    def __init__(self):
        super().__init__('robot_drive')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.cmd_vel = Twist()
        self.my_cmd = Float64MultiArray()
        self.n = 2
        self.p1 = [0.0, 0.0, 0.0] # linear      angular      steering_mode
        self.p2 = [0.0, 0.0, 0.0] # motor_on    motor_off    motor_id
        self.p3 = [0.0, 0.0, 0.0]
        self.joy_rover = Joy()
        self.joy_station = Joy()
        self.steering_mode = 0.0
        self.change_mode_on = False
        self.motor_id = 1.0
        self.change_motor_on = False
        self.joy_rover_on = False
        self.joy_station_on = False
        self.station_joytype = "Unknown"
        self.lost_connect_time = 1.0

        self.joy_rover_subscription = self.create_subscription(
            Joy,
            '/joy_rover',
            self.joy_rover_callback,
            qos_profile,
        )
        self.joy_rover_subscription
        self.last_joy_rover_callback_time = self.get_clock().now()
        self.joy_station_subscription = self.create_subscription(
            Joy,
            '/joy_station',
            self.joy_station_callback,
            qos_profile,
        )
        self.joy_station_subscription
        self.last_joy_station_callback_time = self.get_clock().now()
        self.station_joytype_subscription = self.create_subscription(
            String,
            '/station_joytype',
            self.station_joytype_callback,
            qos_profile,
        )
        self.station_joytype_subscription
        self.publisher_ = self.create_publisher(Float64MultiArray, '/my_cmd', qos_profile)
        self.pub_rover_joytype = self.create_publisher(String, '/rover_joytype', qos_profile)
        self._timer = self.create_timer(0.2, self.publish_my_cmd_vel)

    def joy_station_callback(self, joy_msg):
        self.last_joy_station_callback_time = self.get_clock().now()
        self.joy_station = joy_msg
        self.joy_station_on = True
    
    def station_joytype_callback(self, joytype_msg):
        self.station_joytype = joytype_msg.data

    def joy_rover_callback(self, joy_msg):
        self.last_joy_rover_callback_time = self.get_clock().now()
        self.joy_rover = joy_msg
        self.joy_rover_on = True

    def publish_my_cmd_vel(self):
        joy_msg = Joy()
        if self.joy_rover_on and not self.joy_station_on:
            joy_msg = self.joy_rover
        elif not self.joy_rover_on and self.joy_station_on:
            joy_msg = self.joy_station
        else:
            joy_msg = self.joy_rover
        start_thredthold = 0.01
        connection_type = self.get_joystick_connection_type(0)
        joytype = String()
        joytype.data = connection_type
        self.pub_rover_joytype.publish(joytype)
        if connection_type:
            print(f"Joystick connection type: {connection_type}")
        else:
            print("Unable to determine the joystick connection type.")
        connected = False
        if (connection_type == "Wired (USB)" or self.station_joytype == "Wired (USB)") and (self.joy_rover_on or self.joy_station_on):
            connected = True
            JOY_BTN_A = 0           # motor_on
            JOY_BTN_B = 1           # motor_off
            JOY_BTN_X = 2           # change_motor
            JOY_BTN_Y = 3           # change_mode
            JOY_BTN_LB = 4          # donut_on
            JOY_BTN_RB = 5          # joy_on
            JOY_STICK2_B = 10       
            JOY_STICK1_x = 0
            JOY_STICK1_y = 1        # forword backword
            JOY_BTN_LT = 2
            JOY_STICK2_x = 3        # left right
            JOY_STICK2_y = 4
            JOY_BTN_RT = 5          # speed control 
        elif (connection_type == "Bluetooth (UHID)" or connection_type == "Bluetooth" or self.station_joytype == "Bluetooth (UHID)" or self.station_joytype == "Bluetooth") and (self.joy_rover_on or self.joy_station_on):
            connected = True
            JOY_BTN_A = 0           # motor_on
            JOY_BTN_B = 1           # motor_off
            JOY_BTN_X = 3           # change_motor
            JOY_BTN_Y = 4           # change_mode
            JOY_BTN_LB = 6          # donut_on
            JOY_BTN_RB = 7          # joy_on
            JOY_STICK2_B = 10
            JOY_STICK1_x = 0
            JOY_STICK1_y = 1        # forword backword
            JOY_BTN_LT = 2
            JOY_STICK2_x = 2        # left right
            JOY_STICK2_y = 3
            JOY_BTN_RT = 4          # speed control
        else:
            connected = False
            self.p1[0] = 0.0
            self.p1[1] = 0.0
            self.p1[2] = 0.0
            self.p2[0] = 0.0
            self.p2[1] = 0.0
            self.p2[2] = 1.0

            # JOY_BTN_A = 0           # motor_on
            # JOY_BTN_B = 1           # motor_off
            # JOY_BTN_X = 2           # change_motor
            # JOY_BTN_Y = 3           # change_mode
            # JOY_BTN_LB = 4          # donut_on
            # JOY_BTN_RB = 5          # joy_on
            # JOY_STICK2_B = 10       
            # JOY_STICK1_x = 0
            # JOY_STICK1_y = 1        # forword backword
            # JOY_BTN_LT = 2
            # JOY_STICK2_x = 3        # left right
            # JOY_STICK2_y = 4
            # JOY_BTN_RT = 5          # speed control 
        try:
            if connected:
                joy_on = joy_msg.buttons[JOY_BTN_RB] == 1
                donut_on = joy_msg.buttons[JOY_BTN_LB] == 1
                motor_on = joy_msg.buttons[JOY_BTN_A] == 1
                motor_off = joy_msg.buttons[JOY_BTN_B] == 1
                # steering_on = abs(joy_msg.axes[JOY_STICK2_x]) >= start_thredthold
                change_mode = joy_msg.buttons[JOY_BTN_X] == 1
                change_motor = joy_msg.buttons[JOY_BTN_Y] == 1
                if joy_on:
                    if abs(joy_msg.axes[JOY_STICK1_y]) <= start_thredthold:
                        self.p1[0] = 0.0
                    elif joy_msg.axes[JOY_STICK1_y] > start_thredthold:
                        self.p1[0] = 0.01 * mymap(joy_msg.axes[JOY_BTN_RT],1.0,-1.0, 1.0, (0.2/0.01))
                    elif joy_msg.axes[JOY_STICK1_y] < -start_thredthold:
                        self.p1[0] = -0.01 * mymap(joy_msg.axes[JOY_BTN_RT],1.0,-1.0, 1.0, (0.2/0.01))
                else:
                    self.p1[0] = 0.0
                    self.p1[1] = 0.0
                    # self.p1[2] = 0.0
                    self.p2[0] = 0.0
                    self.p2[1] = 0.0
                    # self.p2[2] = 0.0
                if motor_on:
                    self.p2[0] = 1.0
                else:
                    self.p2[0] = 0.0
                if motor_off:
                    self.p2[1] = 1.0
                else:
                    self.p2[1] = 0.0
                
                if donut_on:
                    self.p1[2] = 2.0
                    if joy_on:
                        if abs(joy_msg.axes[JOY_STICK2_x]) <= start_thredthold:
                            self.p1[1] = 0.00
                        elif joy_msg.axes[JOY_STICK2_x] > start_thredthold:
                            self.p1[1] = interp(abs(joy_msg.axes[JOY_STICK2_x]), [0.0, 1.0], [0.0, 0.10])
                        elif joy_msg.axes[JOY_STICK2_x] < -start_thredthold:
                            self.p1[1] = interp(abs(joy_msg.axes[JOY_STICK2_x]), [0.0, 1.0], [0.0, -0.10])
                else:
                    self.p1[2] = self.steering_mode
                    if joy_on:
                        if abs(joy_msg.axes[JOY_STICK2_x]) <= start_thredthold:
                            self.p1[1] = 0.0
                        elif joy_msg.axes[JOY_STICK2_x] > start_thredthold:
                            self.p1[1] = interp(abs(joy_msg.axes[JOY_STICK2_x]), [0.0, 1.0], [0.0, 1.00 - start_thredthold])
                        elif joy_msg.axes[JOY_STICK2_x] < -start_thredthold:
                            self.p1[1] = interp(abs(joy_msg.axes[JOY_STICK2_x]), [0.0, 1.0], [0.0, -1.00 + start_thredthold])
                
                if change_mode and not donut_on:
                    if not self.change_mode_on:
                        if self.steering_mode == 0.0:
                            self.steering_mode = 1.0
                        elif self.steering_mode == 1.0:
                            self.steering_mode = 3.0
                        elif self.steering_mode == 3.0:
                            self.steering_mode = 0.0
                        self.change_mode_on = True
                else:
                    self.change_mode_on = False

                self.p2[2] = self.motor_id
                if change_motor:
                    if not self.change_motor_on:
                        if self.motor_id == 1.0:
                            self.motor_id = 2.0
                        elif self.motor_id == 2.0:
                            self.motor_id = 3.0
                        elif self.motor_id == 3.0:
                            self.motor_id = 4.0
                        elif self.motor_id == 4.0:
                            self.motor_id = 5.0
                        elif self.motor_id == 5.0:
                            self.motor_id = 6.0
                        elif self.motor_id == 6.0:
                            self.motor_id = 7.0
                        elif self.motor_id == 7.0:
                            self.motor_id = 8.0
                        elif self.motor_id == 8.0:
                            self.motor_id = 1.0
                        self.change_motor_on = True
                else:
                    self.change_motor_on = False
        except:
            self.p1[0] = 0.0
            self.p1[1] = 0.0
            self.p1[2] = 0.0
            self.p2[0] = 0.0
            self.p2[1] = 0.0
            self.p2[2] = 1.0
        
        self.p1[0] = round(self.p1[0], 2)
        self.p1[1] = round(self.p1[1], 2)
        self.my_cmd.data = [*self.p1, *self.p2]
        self.my_cmd.layout.data_offset =  0 # no padding
        dim = []
        dim.append(MultiArrayDimension(label="set", size=self.n, stride=3*self.n))
        dim.append(MultiArrayDimension(label="data", size=3, stride=1))
        self.my_cmd.layout.dim = dim
        self.publisher_.publish(self.my_cmd)

        # Check connection statuses
        now = self.get_clock().now()
        if ((now - self.last_joy_rover_callback_time).nanoseconds / 1e9) >= self.lost_connect_time:  # Convert to seconds
            self.joy_rover_on = False
        if ((now - self.last_joy_station_callback_time).nanoseconds / 1e9) >= self.lost_connect_time:  # Convert to seconds
            self.joy_station_on = False
    
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

    robot_drive = Robot_Drive()

    rclpy.spin(robot_drive)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_drive.destroy_node()
    rclpy.shutdown()

def mymap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if __name__ == '__main__':
    main()