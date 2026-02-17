#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String

import json
import random
import requests
import subprocess

class Station_LoRa(Node):
    def __init__(self):
        super().__init__('station_lora')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        self.motor = ['N/A'] * 8
        self.gps = ['N/A'] * 3

        self.publisher_ = self.create_publisher(String, 'station_lora/receive', qos_profile)
        self._timer = self.create_timer(1.5, self.publish_station_lora)
        
    def generate_random_motor_data(self):
        motor_data = []
        motor_names = ['mg_1', 'mg_2', 'mg_3', 'mg_4', 'sm_5', 'sm_6', 'sm_7', 'sm_8']
        
        for name in motor_names:
            motor = {
                "name": name,
                "effort": round(random.uniform(-1, 1), 6),  # Random effort between -1 and 1
                "position": round(random.uniform(0, 360), 4),  # Random position between 0 and 360 degrees
                "tempareture": random.randint(25, 40),  # Random temperature between 25 and 40 degrees Celsius
                "velocity": round(random.uniform(0, 100), 2),  # Random velocity between 0 and 100
                "voltage": random.randint(110, 120),  # Random voltage between 110 and 120V (for some motors)
                "current": random.randint(1, 30),  # Random current between 1 and 30A
                "move": random.choice([0, 1])  # Random move status (0 = not moving, 1 = moving)
            }
            motor_data.append(motor)

        return motor_data
    
    def get_public_ip(self):
        try:
            ip = subprocess.check_output("curl -s ifconfig.me", shell=True).decode().strip()
            return ip
        except Exception as e:
            print(f"Error getting public IP: {e}")
            return None

    def get_location(self, ip):
        try:
            url = f"http://ip-api.com/json/{ip}"
            response = requests.get(url)
            data = response.json()
            
            if data["status"] == "success":
                self.gps[0] = float(data["lat"])  # Latitude
                self.gps[1] = float(data["lon"])  # Longitude
                self.gps[2] = 0.0  # ไม่มี Altitude ให้ใช้ 0.0
                return self.gps
            else:
                print("Error: Unable to get location")
                return None
        except Exception as e:
            print(f"Error getting location: {e}")
            return None

    def publish_station_lora(self):
        try:
            motor_obj = self.generate_random_motor_data()
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

        ip = self.get_public_ip()
        gps_data = self.get_location(ip)

        status = {
            "temps": {
                "rct": round(random.uniform(30.0, 80.0), 2),  # อุณหภูมิ CPU (30.0 - 80.0 องศา)
                "rht1": round(random.uniform(20.0, 60.0), 2),
                "rht2": round(random.uniform(20.0, 60.0), 2),
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
            "gps": gps_data,
        }

        data = String()
        data.data = json.dumps(status)
        self.publisher_.publish(data)

def main(args=None):
    rclpy.init(args=args)
    node = Station_LoRa()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()