#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import random
import json
from datetime import datetime, timedelta

class SoilDataPublisher(Node):
    def __init__(self):
        super().__init__('soil_data_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        self.publisher_ = self.create_publisher(String, '/soil/rawdata', qos_profile)
        # self.publisher_gps_date = self.create_publisher(String, '/gps/date', 10)
        # self.publisher_gps_time = self.create_publisher(String, '/gps/time', 10)
        self.current_date = datetime.now()
        self.timer = self.create_timer(10.0, self.publish_message)  # ส่งข้อความทุก 10 วินาที

    def publish_message(self):
        # สร้าง dictionary สำหรับ JSON
        soil_data = {
            "misture": round(random.uniform(10.0, 100.0), 2),  # ความชื้นในช่วง 10.0 ถึง 100.0
            "tempe": round(random.uniform(15.0, 35.0), 2),  # อุณหภูมิในช่วง 15.0 ถึง 35.0
            "ec": round(random.uniform(0.1, 5.0), 2),  # Electrical Conductivity ในช่วง 0.1 ถึง 5.0
            "ph": round(random.uniform(4.0, 9.0), 2),  # ค่า pH ในช่วง 4.0 ถึง 9.0
            "n": round(random.uniform(0.0, 200.0), 2),  # ไนโตรเจนในช่วง 0.0 ถึง 200.0
            "p": round(random.uniform(0.0, 200.0), 2),  # ฟอสฟอรัสในช่วง 0.0 ถึง 200.0
            "k": round(random.uniform(0.0, 200.0), 2),  # โพแทสเซียมในช่วง 0.0 ถึง 200.0
        }

        # แปลง dictionary เป็น JSON string
        msg = String(data=json.dumps(soil_data))
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

        # เพิ่มวันไปเรื่อยๆ
        # self.current_date += timedelta(days=1)
        # แปลงวันที่เป็นรูปแบบ DDMMYY
        formatted_date = self.current_date.strftime("%d%m%y")
        gps_date_msg = String()
        gps_date_msg.data = formatted_date
        # self.publisher_gps_date.publish(gps_date_msg)

        # Generate random hours (0 to 23)
        hours = random.randint(0, 23)
        # Generate random minutes (0 to 59)
        minutes = random.randint(0, 59)
        # Generate random seconds (0 to 59)
        seconds = random.randint(0, 59)
        # Combine them into one number in HHMMSS format
        random_time = f"{hours:02d}{minutes:02d}{seconds:02d}"
        # Ensure the time is always 6 digits long
        utc_time = datetime.strptime(random_time, "%H%M%S")
        bangkok_time = utc_time + timedelta(hours=7)
        bangkok_time_str = bangkok_time.strftime("%H%M%S")
        gps_time_msg = String()
        gps_time_msg.data = bangkok_time_str
        # self.publisher_gps_time.publish(gps_time_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SoilDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()