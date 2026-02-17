#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import rclpy.waitable
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

import json
import random
import string
from collections import defaultdict

class Robot_LoRa(Node):
    def __init__(self):
        super().__init__('robot_lora')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        self.rover_health_sub = self.create_subscription(
            String,
            '/rover_health',
            self.rover_health_callback,
            qos_profile)
        self.last_rover_health_callback_time = self.get_clock().now()
        self.motor_sub = self.create_subscription(
            String,
            '/motor_msg/rawdata',
            self.motor_callback,
            qos_profile)
        
        self.last_motor_callback_time = self.get_clock().now()
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            qos_profile)
        self.last_diagnostics_callback_time = self.get_clock().now()

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/rawdata',
            self.gps_callback,
            qos_profile)
        self.last_gps_callback_time = self.get_clock().now()

        self.data = String()
        self.serial_port = None
        # self._timer = self.create_timer(1.0, self.timer_callback)
        self.serial_read_error_count = 0
        self.serial_read_count = 0
        self.lost_connect_time = 5.0
        self.cpu_temp = 'N/A'
        self.temp_1 = 'N/A'
        self.temp_2 = 'N/A'
        self.voltage_1 = 'N/A'
        self.voltage_2 = 'N/A'
        self.motor = ['N/A'] * 8
        self.imu = ['N/A'] * 10
        self.gps = ['N/A'] * 3

        self.publisher_ = self.create_publisher(String, 'robot_lora/send', qos_profile)
        self._timer = self.create_timer(0.2, self.publish_send)

    def gps_callback(self, msg):
        try:
            self.gps[0] = msg.latitude
            self.gps[1] = msg.longitude
            self.gps[2] = msg.altitude
            self.last_gps_callback_time = self.get_clock().now()
        except:
            pass

    def imu_callback(self, msg):
        try:
            self.imu[0] = msg.orientation.x
            self.imu[1] = msg.orientation.y
            self.imu[2] = msg.orientation.z
            self.imu[3] = msg.orientation.w
            self.imu[4] = msg.angular_velocity.x
            self.imu[5] = msg.angular_velocity.y
            self.imu[6] = msg.angular_velocity.z
            self.imu[7] = msg.linear_acceleration.x
            self.imu[8] = msg.linear_acceleration.y
            self.imu[9] = msg.linear_acceleration.z
            self.last_imu_callback_time = self.get_clock().now()
        except:
            pass

    def rover_health_callback(self, msg):
        try:
            rover_health_obj = json.loads(msg.data)  # แปลง JSON
            # อ่านค่า temp1 และ temp2
            self.temp_1 = round(rover_health_obj.get("temp1", "N/A"), 2)
            self.temp_2 = round(rover_health_obj.get("temp2", "N/A"), 2)
            self.voltage_1 = round(rover_health_obj.get("voltage1", "N/A"), 2) 
            self.voltage_2 = round(rover_health_obj.get("voltage2", "N/A"), 2) 
            '''  jsonDoc["voltage1"] = PZEMVoltage1;jsonDoc["current1"] = PZEMCurrent1; jsonDoc["power1"] = PZEMPower1;jsonDoc["energy1"] = PZEMEnergy1;jsonDoc["voltage2"] = PZEMVoltage2;jsonDoc["current2"] = PZEMCurrent2; jsonDoc["power2"] = PZEMPower2; jsonDoc["energy2"] = PZEMEnergy2;'''


            # ตรวจสอบค่า
            print(f"Temp1: {self.temp_1}, Temp2: {self.temp_2}")
            # อัปเดตเวลาล่าสุด
            self.last_rover_health_callback_time = self.get_clock().now()
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON temp: {e}")
    
    def motor_callback(self, msg):
        try:
            motor_obj = json.loads(msg.data)
            motor_status = []  # สร้างอาร์เรย์เก็บสถานะมอเตอร์

            for motor in motor_obj:
                effort = motor.get("effort", -1)
                velocity = motor.get("velocity", -1)
                move = motor.get("move", -1)  # บางมอเตอร์อาจไม่มี move

                # ตรวจสอบว่ามอเตอร์ทำงานหรือไม่
                if effort not in [0, -1] or velocity not in [0, -1] or move not in [0, -1]:
                    motor_status.append(1)  # มอเตอร์ทำงาน
                else:
                    motor_status.append(0)  # มอเตอร์ไม่ทำงาน

            self.motor = motor_status  # self.motor เป็นอาร์เรย์ของสถานะมอเตอร์
            self.last_motor_callback_time = self.get_clock().now()
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON motor: {e}")
        except Exception as e:
            print(f"Unexpected error in motor_callback: {e}")

    def diagnostics_callback(self, msg):
        try:
            for status in msg.status:
                if "Sensor Status" in status.name:
                    temp_values = [float(v.value) for v in status.values if "Temperature" in v.key]
                    
                    if temp_values:
                        self.cpu_temp = round(sum(temp_values) / len(temp_values), 2)
                    else:
                        self.cpu_temp = -1.0
                        print("Error: No temperature values found in diagnostic message")

                    self.last_diagnostics_callback_time = self.get_clock().now()
                    break  # หยุดการวนลูปเมื่อพบ Sensor Status แล้ว
        except ValueError:
            print("Error: Cannot convert temperature values to float")

    def get_status(self):
        try:
            # ฟังก์ชันสำหรับสุ่มตัวเลข 3 หลักก่อนและหลังจุดทศนิยม
            def generate_random_number():
                integer_part = ''.join(random.choices("0123456789", k=1))
                decimal_part = ''.join(random.choices("0123456789", k=2))
                return f"{integer_part}"
                # return f"{integer_part}.{decimal_part}"
            # สร้าง JSON ด้วย key mydata_1 ถึง mydata_50
            random.choice(string.ascii_letters)
            data = {f"{i}": generate_random_number() for i in range(0, 10)}
            # status = data
            status = {
                "temps": {
                    "rct": self.cpu_temp,  # robot_cpu_temp
                    "rht1": self.temp_1,
                    "rht2": self.temp_2,
                    "vo1":self.voltage_1,
                    "vo2":self.voltage_2,
                },
                "motor": {
                    "wheel_fr": self.motor[0],  # robot_motor 1
                    "wheel_fl": self.motor[1],
                    "wheel_rr": self.motor[2],
                    "wheel_rl": self.motor[3],
                    "str_fr": self.motor[4],
                    "str_fl": self.motor[5],
                    "str_rr": self.motor[6],
                    "str_rl": self.motor[7],
                },
                "gps": self.gps,
            }
            return json.dumps(status)
        except Exception as e:
            print(f"Error in get_status: {e}")
            return None

    def publish_send(self):
        self.data.data = self.get_status()
        self.publisher_.publish(self.data)

def main(args=None):
    rclpy.init(args=args)
    node = Robot_LoRa()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# +++\r\n                                               Enter AT command mode
# AT+EXIT\r\n                                           Exit AT command mode
# ATE\r\n                                               Enable/disable AT command echo
# AT+VER\r\n                                            Check the software version number
# AT+HELP\r\n                                           View AT help

# AT+SF=7\r\n                                           Set the spreading factor to 7, the value range is 7~12
# AT+BW=0\r\n                                           Set bandwidth, 0 means 125KHz, 1 means 250KHz, 2 means 500KHz
# AT+CR=1\r\n                                           Set the encoding rate to 1, 1 represents 4/5, 2 represents 4/6, 3 represents 4/7, 4 represents 4/8
# AT+PWR=22\r\n                                         Set the RF power, the value range is 10~22dBm
# AT+NETID=0\r\n                                        Network ID assignment, the value range is 0~65535
# AT+LBT=0\r\n                                          Enable/disable LBT function, 0: disable, 1: enable
# AT+MODE=1                                             DTU working mode, 1: stream mode, 2: packet mode, 3: relay mode
# AT+TXCH=18\r\n                                        Transmit channel, value range 0~80, corresponding frequency point is 850~930MHz or 410~490MHz
# AT+RXCH=18\r\n                                        Receive channel, value range 0~80, corresponding frequency point is 850~930MHz or 410~490MHz
# AT+RSSI=0\r\n                                         Enable/disable RSSI signal value output, 0: disable, 1: enable
# AT+ADDR=0\r\n                                         Set DTU address, value range 0~65535
# AT+PORT=3\r\n                                         Set COM port, 1:RS422, 2:RS485, 3:RS232
# AT+BAUD=115200\r\n                                    Set COMx port baud rate, value range 1200~115200, 1200, 2400, ....., 57600, 115200ss
# AT+COMM="8N1"\r\n                                     Set COM port parameters, data bits: 8 or 9, parity: N, O, E, stop bits: 0, 1, 2
# AT+AllP=7,2,1,22,0,0,1,18,18,0,0,3,115200,"8N1",0   Set the spreading factor to key multi-parameter
# AT+RESTORE=0\r\n                                      Restore factory settings, 0: disabled, 1: enabled