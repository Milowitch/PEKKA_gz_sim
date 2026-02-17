#!/usr/bin/python3
import math
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray, MultiArrayDimension
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JoyFeedback
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import os
import time

vel_msg = Twist()  # robot velosity
mode_selection = 4 # 1:opposite phase, 2:in-phase, 3:pivot turn 4: none
donut = True
donut_auto = False
speed = 0.4
donut_speed = 0.07

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE
)

reliable_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE
)

def calculate_rpm(speed_mps, wheel_radius):
    return (speed_mps * 60.0) / (2.0 * math.pi * wheel_radius)

class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        self.declare_parameter('speed', 0.4)            # Double parameter
        self.declare_parameter('donut_speed', 0.07)      # Double parameter
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.donut_speed = self.get_parameter('donut_speed').get_parameter_value().double_value

        global speed, donut_speed
        speed = self.speed
        donut_speed = self.donut_speed

        timer_period = 0.02
        self.wheel_seperation = 0.43
        self.wheel_base = 0.51
        self.wheel_radius = 0.127
        self.wheel_steering_y_offset = 0.01
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset

        self.pos = np.array([0,0,0,0], float)
        self.vel = np.array([0,0,0,0], float) #left_front, right_front, left_rear, right_rear
        self.my_cmd = np.array([0,0,0,0,0,0,0,0], float)
        self.current_pos_0 = 0
        self.current_pos_1 = 0

        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', qos_profile)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', qos_profile)
        self.publisher_ = self.create_publisher(Float64MultiArray, '/my_cmd', reliable_profile) # vel x = RPM
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self._timer = self.create_timer(0.2, self.publish_my_cmd_vel)

    def timer_callback(self):
        global vel_msg, mode_selection

        # opposite phase
        if mode_selection == 1:
            vel_steerring_offset = vel_msg.angular.z * self.wheel_steering_y_offset
            sign = np.sign(vel_msg.linear.x)

            v0 = math.hypot(abs(vel_msg.linear.x) - vel_msg.angular.z * self.steering_track / 2, vel_msg.angular.z * self.wheel_base / 2) - vel_steerring_offset
            v1 = math.hypot(abs(vel_msg.linear.x) + vel_msg.angular.z * self.steering_track / 2, vel_msg.angular.z * self.wheel_base / 2) + vel_steerring_offset
            v2 = math.hypot(abs(vel_msg.linear.x) - vel_msg.angular.z * self.steering_track / 2, vel_msg.angular.z * self.wheel_base / 2) - vel_steerring_offset
            v3 = math.hypot(abs(vel_msg.linear.x) + vel_msg.angular.z * self.steering_track / 2, vel_msg.angular.z * self.wheel_base / 2) + vel_steerring_offset

            max_wheel_speed = max(abs(v0), abs(v1), abs(v2), abs(v3), 1e-6)
            scale_factor = min(1.0, math.hypot(vel_msg.linear.x, vel_msg.angular.z * self.wheel_base) / max_wheel_speed)

            if abs(vel_msg.linear.x) > 0.01:
                self.vel[0] = sign * v0 * scale_factor
                self.vel[1] = sign * v1 * scale_factor
                self.vel[2] = sign * v2 * scale_factor
                self.vel[3] = sign * v3 * scale_factor
            else:
                self.vel[0] = 0
                self.vel[1] = 0
                self.vel[2] = 0
                self.vel[3] = 0

            if abs(vel_msg.angular.z) > 1e-6:
                radius = 1.0 / vel_msg.angular.z
                self.pos[0] = math.atan(self.wheel_base / (radius - self.steering_track / 2)) + vel_steerring_offset
                self.pos[1] = math.atan(self.wheel_base / (radius + self.steering_track / 2)) + vel_steerring_offset
            else:
                self.pos[0] = 0
                self.pos[1] = 0

            self.pos[2] = -self.pos[0]
            self.pos[3] = -self.pos[1]

        # # In-phase
        # elif(mode_selection == 2):
        #     V = math.hypot(vel_msg.linear.x, vel_msg.linear.y)
        #     sign = np.sign(vel_msg.linear.x)

        #     if abs(vel_msg.linear.x) < 0.3:
        #         if vel_msg.linear.y > 0.2:
        #             target_pos = math.pi / 2
        #             self.vel[:] = V
        #         elif vel_msg.linear.y < -0.2:
        #             target_pos = -math.pi / 2
        #             self.vel[:] = V
        #         else:
        #             target_pos = 0
        #     else:
        #         ang = vel_msg.linear.y / vel_msg.linear.x
        #         target_pos = math.atan(ang)
        #         self.vel[:] = sign * V

        #     # Gradually adjust current_pos towards target_pos
        #     lerp_factor = 1.0  # Adjust how fast the change happens
        #     self.current_pos_0 += lerp_factor * (target_pos - self.current_pos_0)
        #     self.current_pos_1 += lerp_factor * (target_pos - self.current_pos_1)

        #     # Update self.pos
        #     self.pos[0] = self.current_pos_0
        #     self.pos[1] = self.current_pos_1
        #     self.pos[2] = self.pos[0]
        #     self.pos[3] = self.pos[1]

        #     # self.get_logger().info(f"sign: {sign:.2f}, V: {V:.2f}")
        #     # self.get_logger().info(f"Target pos: {target_pos*180/math.pi:.2f}, Current pos: {self.pos[0]*180/math.pi:.2f}")
            
        # pivot turn
        elif(mode_selection == 3):

            self.pos[0] = -math.atan(self.wheel_base/self.steering_track)
            self.pos[1] = math.atan(self.wheel_base/self.steering_track)
            self.pos[2] = math.atan(self.wheel_base/self.steering_track)
            self.pos[3] = -math.atan(self.wheel_base/self.steering_track)

            if abs(vel_msg.angular.z) > 0.01:
                self.vel[0] = -vel_msg.angular.z
                self.vel[1] = vel_msg.angular.z
                self.vel[2] = self.vel[0]
                self.vel[3] = self.vel[1]
            else:
                self.vel[0] = 0
                self.vel[1] = 0
                self.vel[2] = self.vel[0]
                self.vel[3] = self.vel[1]
        else:
            self.pos[:] = 0
            self.vel[:] = 0

        pos_array = Float64MultiArray(data=self.pos) 
        vel_array = Float64MultiArray(data=self.vel)
        self.pub_pos.publish(pos_array)
        self.pub_vel.publish(vel_array)

        global donut, donut_speed
        # self.my_cmd = np.hstack((self.vel, self.pos))
        multi_speed = 2.67
        if mode_selection == 3:
            self.my_cmd[0] = calculate_rpm(np.clip(self.vel[0], -donut_speed, donut_speed)*multi_speed, self.wheel_radius)
            self.my_cmd[1] = calculate_rpm(np.clip(self.vel[1], -donut_speed, donut_speed)*multi_speed, self.wheel_radius)
            self.my_cmd[2] = calculate_rpm(np.clip(self.vel[2], -donut_speed, donut_speed)*multi_speed, self.wheel_radius)
            self.my_cmd[3] = calculate_rpm(np.clip(self.vel[3], -donut_speed, donut_speed)*multi_speed, self.wheel_radius)
            if donut:
                self.my_cmd[4] = self.pos[0]
                self.my_cmd[5] = self.pos[1]
                self.my_cmd[6] = self.pos[2]
                self.my_cmd[7] = self.pos[3]
        else:
            self.my_cmd[0] = calculate_rpm(self.vel[0]*multi_speed, self.wheel_radius)
            self.my_cmd[1] = calculate_rpm(self.vel[1]*multi_speed, self.wheel_radius)
            self.my_cmd[2] = calculate_rpm(self.vel[2]*multi_speed, self.wheel_radius)
            self.my_cmd[3] = calculate_rpm(self.vel[3]*multi_speed, self.wheel_radius)
            if donut:
                self.my_cmd[4] = self.pos[0]
                self.my_cmd[5] = self.pos[1]
                self.my_cmd[6] = self.pos[2]
                self.my_cmd[7] = self.pos[3]

        self.pos[:] = 0
        self.vel[:] = 0
    
    def publish_my_cmd_vel(self):
        my_cmd_array = Float64MultiArray(data=self.my_cmd)
        self.publisher_.publish(my_cmd_array)
        self.my_cmd[:] = 0
        # my_cmd_array = Float32MultiArray()
        # self.n = 2
        # self.p1 = [0.0, 0.0, 0.0]
        # self.p2 = [0.0, 0.0, 0.0]
        # self.p3 = [0.0, 0.0, 0.0]
        # my_cmd_array.data = [*self.p1, *self.p2]
        # my_cmd_array.layout.data_offset =  0 # no padding
        # dim = []
        # dim.append(MultiArrayDimension(label="set", size=self.n, stride=3*self.n))
        # dim.append(MultiArrayDimension(label="data", size=3, stride=1))
        # my_cmd_array.layout.dim = dim
        # self.publisher_.publish(my_cmd_array)
        # self.my_cmd[:] = 0

class Joy_subscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')
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
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            reliable_profile,
        )
        self.mode_now = 0
        self.last_mode_change_time = self.get_clock().now()

        self.pub_input_at_waypoint= self.create_publisher(Empty, '/input_at_waypoint/input', reliable_profile)
        self.pub_joy_station_feedback = self.create_publisher(JoyFeedback, '/joy_station/set_feedback', reliable_profile)
        self.pub_joy_rover_feedback = self.create_publisher(JoyFeedback, '/joy_rover/set_feedback', reliable_profile)

        self.client_soil = self.create_client(Trigger, '/soil_trigger_service')
        self.client_lifecycle_manager_navigation = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')

        self.joy_rover_on = False
        self.joy_station_on = False
        self.station_joytype = "Unknown"
        self.joy_rover = Joy()
        self.joy_station = Joy()
        self.joy_active = False
        self.declare_parameter('js', 2)
        self.pub_rover_joytype = self.create_publisher(String, '/rover_joytype', qos_profile)
        self._timer = self.create_timer(0.2, self.my_publish)
    
    def lifecycle_manager_navigation_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service response: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def call_soil_service(self):
        request = Trigger.Request()
        future = self.client_soil.call_async(request)
        future.add_done_callback(self.callback)
    
    def callback(self, future):
        try:
            response = future.result()  # รับผลลัพธ์จาก service
            if response.success:
                self.get_logger().info('Service call was successful.')
            else:
                self.get_logger().warn('Service call was not successful.')
        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}")
            # ถ้าล้มเหลว, ให้ไปต่อ, ไม่หยุดการทำงานของโปรแกรม
            pass

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

    def adjust_min(self, value, min=0.03):
        if 0 < value < min:
            return min
        elif -min < value < 0:
            return -min
        return value

    def cmd_vel_callback(self, cmd_vel_msg):
        global vel_msg, mode_selection, donut, donut_auto
        now = self.get_clock().now()
        linear_x = np.clip(cmd_vel_msg.linear.x, -0.2, 0.2)

        if donut:
            if donut_auto:
                angular_z = np.clip(cmd_vel_msg.angular.z*1.5, -1.0, 1.0)
            else:
                angular_z = np.clip(cmd_vel_msg.angular.z*1.5, -0.599, 0.599)
        else:
            angular_z = np.clip(cmd_vel_msg.angular.z*1.5, -0.599, 0.599)

        linear_x = self.adjust_min(value=linear_x)
        angular_z = self.adjust_min(value=angular_z)

        if self.joy_active:
            return

        if abs(angular_z) > 0.6:
            self.mode_now = 3
            mode_selection = 3
            self.last_mode_change_time = now  # Save time when entering mode 3
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = cmd_vel_msg.linear.z
            vel_msg.angular.x = cmd_vel_msg.angular.x
            vel_msg.angular.y = cmd_vel_msg.angular.y
            vel_msg.angular.z = angular_z
        else:
            if self.mode_now == 3:
                elapsed_time = (now - self.last_mode_change_time).nanoseconds / 1e9  # Convert to seconds
                if elapsed_time < 1.0:
                    return  # Wait until 1 seconds have passed before switching modes
            self.mode_now = 1
            mode_selection = 1
            if linear_x == 0:
                linear_x = 0.03
            vel_msg.linear.x = linear_x
            vel_msg.linear.y = cmd_vel_msg.linear.y
            vel_msg.linear.z = cmd_vel_msg.linear.z
            vel_msg.angular.x = cmd_vel_msg.angular.x
            vel_msg.angular.y = cmd_vel_msg.angular.y
            vel_msg.angular.z = np.clip(angular_z, -0.6, 0.6)
    
    def my_publish(self):
        global vel_msg, mode_selection

        joy_msg = Joy()
        if self.joy_rover_on and not self.joy_station_on:
            joy_msg = self.joy_rover
        elif not self.joy_rover_on and self.joy_station_on:
            joy_msg = self.joy_station
        elif self.joy_rover_on and self.joy_station_on:
            joy_msg = self.joy_rover
        else:
            joy_msg.axes = [0.0] * len(joy_msg.axes)  # Set all axes to 0.0
            joy_msg.buttons = [0] * len(joy_msg.buttons)  # Set all buttons to 0
            pass
        connection_type = self.get_joystick_connection_type(self.get_parameter('js').value)
        joytype = String()
        joytype.data = connection_type
        self.pub_rover_joytype.publish(joytype)

        # if connection_type:
        #     print(f"Joystick connection type: {connection_type}")
        # else:
        #     print("Unable to determine the joystick connection type.")
        connected = False
        if (connection_type == "Wired (USB)" or self.station_joytype == "Wired (USB)"):
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
            JOY_PGUPDN = 7
            JOY_BTN_RT = 5          # speed control 
        elif (connection_type == "Bluetooth (UHID)" or connection_type == "Bluetooth" or self.station_joytype == "Bluetooth (UHID)" or self.station_joytype == "Bluetooth"):
            connected = True
            JOY_BTN_A = 0           # motor_on
            JOY_BTN_B = 1           # motor_off
            JOY_BTN_X = 3           # change_motor
            JOY_BTN_Y = 4           # change_mode
            JOY_BTN_LB = 9          # donut_on
            JOY_BTN_RB = 10          # joy_on
            JOY_STICK2_B = 8
            JOY_STICK1_x = 0
            JOY_STICK1_y = 1        # forword backword
            JOY_BTN_LT = 2
            JOY_STICK2_x = 2        # left right
            JOY_STICK2_y = 3
            JOY_PGUPDN = 7
            JOY_BTN_RT = 4          # speed control
        else:
            connected = False
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.angular.z = 0.0
            mode_selection = 4
        
        try:
            if connected:
                feedback = JoyFeedback()
                feedback.type = 1
                feedback.id = 0
                feedback.intensity = 0.1

                if(joy_msg.buttons[JOY_BTN_A] == 1):   # in-phase # A button of Xbox 360 controller
                    mode_selection = 1 # 2
                    self.joy_active = True
                elif(joy_msg.buttons[JOY_BTN_LB] == 1): # opposite phase # LB button of Xbox 360 controller
                    mode_selection = 1
                    self.joy_active = True
                elif(joy_msg.buttons[JOY_BTN_RB] == 1): # pivot turn # RB button of Xbox 360 controller
                    mode_selection = 3
                    self.joy_active = True
                else:
                    mode_selection = 4
                    self.joy_active = False

                if(joy_msg.axes[JOY_PGUPDN] == -1):
                    self.joy_active = False
                    if(joy_msg.buttons[JOY_BTN_X] == 1):
                        self.call_soil_service()
                        self.pub_joy_station_feedback.publish(feedback)
                        self.pub_joy_rover_feedback.publish(feedback)
                
                if(joy_msg.axes[JOY_PGUPDN] == 1):
                    self.joy_active = False
                    if(joy_msg.buttons[JOY_BTN_Y] == 1):
                        input = Empty()
                        self.pub_input_at_waypoint.publish(input)
                        self.pub_joy_station_feedback.publish(feedback)
                        self.pub_joy_rover_feedback.publish(feedback)
                    
                    if(joy_msg.buttons[JOY_BTN_B] == 1):
                        request = ManageLifecycleNodes.Request()
                        request.command = 1
                        future = self.client_lifecycle_manager_navigation.call_async(request)
                        future.add_done_callback(self.lifecycle_manager_navigation_response)
                        self.pub_joy_station_feedback.publish(feedback)
                        self.pub_joy_rover_feedback.publish(feedback)

                    if(joy_msg.buttons[JOY_BTN_A] == 1):
                        request = ManageLifecycleNodes.Request()
                        request.command = 2
                        future = self.client_lifecycle_manager_navigation.call_async(request)
                        future.add_done_callback(self.lifecycle_manager_navigation_response)
                        self.pub_joy_station_feedback.publish(feedback)
                        self.pub_joy_rover_feedback.publish(feedback)

                if self.joy_active:
                    global speed
                    vel_msg.linear.x = joy_msg.axes[JOY_STICK1_y]*speed
                    vel_msg.linear.y = joy_msg.axes[JOY_STICK1_x]*speed
                    vel_msg.angular.z = joy_msg.axes[JOY_STICK2_x]*0.6
            # else:
            #     vel_msg.linear.x = 0.0
            #     vel_msg.linear.y = 0.0
            #     vel_msg.angular.z = 0.0
        except:
            print("Ohno")
            pass

        now = self.get_clock().now()
        if ((now - self.last_joy_rover_callback_time).nanoseconds / 1e9) >= 1.0:
            self.joy_rover_on = False
            # print("joy_node_0 haven't topic /joy_rover")
            pass
        if ((now - self.last_joy_station_callback_time).nanoseconds / 1e9) >= 1.0:
            self.joy_station_on = False
            mode_selection = 4
            # print("joy_node_0 haven't topic /joy_rover")
            pass
        
    def get_joystick_connection_type(self, joystick_id=0):
        # Path to the joystick device
        device_path = f"/sys/class/input/js{joystick_id}/device"
        
        if not os.path.exists(device_path):
            # print(f"Joystick {joystick_id} not found.")
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
            # print(f"Could not identify connection type for path: {parent_path}")  # Debugging line
            return "Unknown"

def main(args=None):
    rclpy.init(args=None)
    
    commander = Commander()
    joy_subscriber = Joy_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()