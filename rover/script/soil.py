import rclpy
from rclpy.node import Node
from rclpy.service import Service
from example_interfaces.srv import Trigger  # ใช้ service แบบ Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import csv
import os
import json
from datetime import datetime

# สร้างโฟลเดอร์ ~/soil ถ้ายังไม่มี
output_dir = os.path.expanduser("~/files")
os.makedirs(output_dir, exist_ok=True)

# ตัวแปรเก็บข้อมูลล่าสุดจาก soil และ gps
last_soil = None
last_gps = None
gps_date = None
gps_time = None

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE
)

# ฟังก์ชันสำหรับการสร้างชื่อไฟล์
def create_filename(date):
    # เปลี่ยน format ของวันที่เป็นสำหรับไฟล์ (ไม่สามารถใช้ ':' หรือ '/' ในชื่อไฟล์)
    filename_time = f"{date.replace('-', '_')}"
    return os.path.join(output_dir, f"{filename_time}.csv")

# ฟังก์ชันเพิ่มข้อมูลทีละบรรทัด พร้อมเขียนหัวตารางในครั้งแรก
def add_row(data, date, time):
    # ใช้วันที่และเวลา จากข้อมูล /gps/date และ /gps/time
    data_with_datetime = [date, time] + data  # เพิ่มวันที่และเวลาไปด้านหน้า
    # สร้างชื่อไฟล์จากข้อมูลวันที่
    filename = create_filename(date)
    # หัวตาราง
    header = ["Date", "Time", "pH", "Temp", "Moisture", "EC", "N", "P", "K", "Latitude", "Longitude", "Altitude"]
    # ตรวจสอบว่ามีไฟล์อยู่แล้วหรือไม่
    file_exists = os.path.exists(filename)
    # เปิดไฟล์และเขียนข้อมูล
    with open(filename, mode="a", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        # เขียน header หากไฟล์ยังไม่มี
        if not file_exists:
            writer.writerow(header)
            print(f"สร้างไฟล์ใหม่พร้อมหัวตาราง: {header}")
        # เพิ่มข้อมูลในแต่ละบรรทัด
        writer.writerow(data_with_datetime)
        print(f"เพิ่มข้อมูล {data_with_datetime} ลงในไฟล์ {filename} เรียบร้อยแล้ว!")

# ROS 2 Node สำหรับ subscribe ข้อมูลและจัดการ service
class GPSDataService(Node):
    def __init__(self):
        super().__init__("gps_data_service")
        
        # subscribe ข้อมูลจาก /soil/rawdata
        self.subscription_soil = self.create_subscription(
            String,
            "/soil/rawdata",
            self.listener_soil_callback,
            qos_profile
        )
        
        # subscribe ข้อมูลจาก /gps/rawdata
        self.subscription_gps = self.create_subscription(
            NavSatFix,
            "/gps/rawdata",
            self.listener_gps_callback,
            qos_profile
        )

        # subscribe ข้อมูลจาก /gps/date และ /gps/time
        self.subscription_date = self.create_subscription(
            String,
            "/gps/date",
            self.listener_date_callback,
            qos_profile
        )
        
        self.subscription_time = self.create_subscription(
            String,
            "/gps/time",
            self.listener_time_callback,
            qos_profile
        )

    def listener_soil_callback(self, msg):
        global last_soil
        try:
            # แปลง JSON string เป็น dictionary
            soil_data = json.loads(msg.data)
            # เก็บค่าจาก JSON
            last_soil = [
                soil_data.get("ph", None),
                soil_data.get("tempe", None),
                soil_data.get("misture", None),
                soil_data.get("ec", None),
                soil_data.get("n", None),
                soil_data.get("p", None),
                soil_data.get("k", None)
            ]
            # self.get_logger().info(f"รับข้อมูลจาก /soil/rawdata: {last_soil}")
            # ตรวจสอบว่ามีข้อมูลทั้งจาก soil และ gps หรือไม่
            if last_soil is None or last_gps is None or gps_date is None or gps_time is None:
                self.get_logger().info("ยังไม่ได้รับข้อมูลครบถ้วนจาก /soil/rawdata หรือ /gps/rawdata")

            # ตรวจสอบไฟล์ว่ามีหรือไม่ และเป็นวันที่เดียวกันหรือไม่
            filename = create_filename(gps_date)

            if os.path.exists(filename):
                # ถ้ามีไฟล์แล้ว ตรวจสอบว่าเป็นวันที่เดียวกับวันนี้หรือไม่
                file_date = filename.split('/')[-1].split('.')[0]  # แยกชื่อไฟล์เพื่อดึงวันที่
                if file_date == gps_date.replace('-', '_'):
                    # ถ้าเป็นวันที่เดียวกัน ก็เพิ่มข้อมูล
                    self.add_soil_data_to_file(filename)
                    self.get_logger().info(f"เพิ่มข้อมูลลงในไฟล์ {filename} แล้ว")
                else:
                    # ถ้าไม่ใช่วันเดียวกัน สร้างไฟล์ใหม่
                    filename = create_filename(gps_date.replace('-', '_'))
                    self.add_soil_data_to_file(filename)
                    self.get_logger().info(f"สร้างไฟล์ใหม่และเพิ่มข้อมูลลงไปใน {filename}")
            else:
                # ถ้าไม่มีไฟล์ ให้สร้างไฟล์ใหม่
                self.add_soil_data_to_file(filename)
                self.get_logger().info(f"สร้างไฟล์ใหม่และเพิ่มข้อมูลลงไปใน {filename}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"เกิดข้อผิดพลาดในการแปลง JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"เกิดข้อผิดพลาดใน listener_soil_callback: {e}")

    def listener_gps_callback(self, msg):
        global last_gps
        try:
            # รับข้อมูลจาก NavSatFix
            last_gps = [
                msg.latitude,  # Latitude
                msg.longitude,  # Longitude
                msg.altitude   # Altitude
            ]
            # self.get_logger().info(f"รับข้อมูลจาก /gps/rawdata: {last_gps}")
        except Exception as e:
            self.get_logger().error(f"เกิดข้อผิดพลาดใน listener_gps_callback: {e}")

    def listener_date_callback(self, msg):
        global gps_date
        raw_date_str = str(msg.data).strip()
        
        if len(raw_date_str) == 5:  # DMMYY (วันหลักเดียว)
            day = int(raw_date_str[0])  
            month = int(raw_date_str[1:3])  
            year = int(raw_date_str[3:])  
        elif len(raw_date_str) == 6:  # DDMMYY (วันสองหลัก)
            day = int(raw_date_str[:2])  
            month = int(raw_date_str[2:4])  
            year = int(raw_date_str[4:])  
        else:
            self.get_logger().error(f"รูปแบบวันที่ไม่ถูกต้อง: {raw_date_str}")
            return  # ออกจากฟังก์ชันก่อนใช้งาน year

        year += 2000 if year < 100 else 1900  
        formatted_date = f"{year:04d}-{month:02d}-{day:02d}"
        gps_date = formatted_date

    def listener_time_callback(self, msg):
        global gps_time
        raw_time_str = str(msg.data).strip()
        if len(raw_time_str) == 6:  # ถ้าเป็นรูปแบบ hhmmss
            # แยกชั่วโมง, นาที, วินาที
            hours = raw_time_str[:2]
            minutes = raw_time_str[2:4]
            seconds = raw_time_str[4:]
            # จัดรูปแบบใหม่เป็น hh:mm:ss
            formatted_time = f"{hours}:{minutes}:{seconds}"
        else:
            pass
        gps_time = formatted_time
        # self.get_logger().info(f"รับข้อมูลจาก /gps/time: {gps_time}")

    def add_soil_data_to_file(self, filename):
        # เพิ่มข้อมูลลงในไฟล์
        data = last_soil + last_gps  # รวมข้อมูลจาก soil และ gps
        add_row(data, gps_date, gps_time)

def main(args=None):
    rclpy.init(args=args)
    node = GPSDataService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()