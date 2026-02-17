import os
import paramiko
import rclpy
import hashlib
from rclpy.node import Node
from std_msgs.msg import String

class FileSenderNode(Node):
    def __init__(self):
        super().__init__('file_sender_node')

        # ตัวอย่างการตั้งค่า
        self.folder_path = os.path.expanduser('/home/rover/files')  # โฟลเดอร์ที่มีไฟล์ที่ต้องการส่ง
        self.host = '192.168.1.101'  # ไอพีของเครื่องที่ต้องการส่งไฟล์ไป 192.168.1.101
        self.port = 22  # พอร์ตของ SSH
        self.username = 'station'  # ชื่อผู้ใช้ SSH
        self.password = '1'  # รหัสผ่าน SSH
        self.remote_path = os.path.expanduser('/home/station/received_files')  # โฟลเดอร์ปลายทางบนเครื่องปลายทาง

    def calculate_md5(self, file_path):
        """ คำนวณค่า MD5 ของไฟล์ """
        hash_md5 = hashlib.md5()
        try:
            with open(file_path, "rb") as f:
                for chunk in iter(lambda: f.read(4096), b""):
                    hash_md5.update(chunk)
            return hash_md5.hexdigest()
        except Exception as e:
            self.get_logger().error(f"ไม่สามารถคำนวณ MD5 ของ {file_path}: {e}")
            return None

    def monitor_and_resend_files(self, retry_interval=30):
        while True:  # ตรวจสอบอย่างต่อเนื่อง
            transport = None
            sftp = None
            try:
                # สร้างการเชื่อมต่อ SFTP
                self.get_logger().info(f"กำลังเชื่อมต่อไปยัง {self.host}:{self.port}")
                transport = paramiko.Transport((self.host, self.port))
                transport.connect(username=self.username, password=self.password)
                sftp = paramiko.SFTPClient.from_transport(transport)

                # ตรวจสอบโฟลเดอร์ปลายทาง
                try:
                    remote_files = sftp.listdir(self.remote_path)
                except FileNotFoundError:
                    self.get_logger().warning(f"โฟลเดอร์ {self.remote_path} ไม่พบ กำลังสร้างใหม่...")
                    sftp.mkdir(self.remote_path)
                    remote_files = []

                # ตรวจสอบไฟล์ทั้งหมดในโฟลเดอร์ต้นทาง
                for file_name in os.listdir(self.folder_path):
                    local_file_path = os.path.join(self.folder_path, file_name)
                    remote_file_path = os.path.join(self.remote_path, file_name)

                    # ตรวจสอบว่าไฟล์มีอยู่แล้วหรือไม่
                    if file_name in remote_files:
                        try:
                            remote_size = sftp.stat(remote_file_path).st_size
                            local_size = os.path.getsize(local_file_path)

                            # ถ้าไฟล์มีขนาดต่างกัน ให้ส่งไฟล์ใหม่
                            if local_size != remote_size:
                                self.get_logger().info(f"ขนาดไฟล์ {file_name} ไม่ตรงกัน กำลังอัปโหลดใหม่...")
                                sftp.put(local_file_path, remote_file_path)
                                continue

                            # ตรวจสอบค่า MD5
                            local_md5 = self.calculate_md5(local_file_path)
                            remote_md5 = self.calculate_md5(remote_file_path)

                            if local_md5 != remote_md5:
                                self.get_logger().info(f"เนื้อหาของ {file_name} เปลี่ยนแปลง กำลังอัปโหลดใหม่...")
                                sftp.put(local_file_path, remote_file_path)
                            else:
                                self.get_logger().info(f"ไฟล์ {file_name} ไม่มีการเปลี่ยนแปลง ไม่ต้องอัปโหลดใหม่")
                        except Exception as e:
                            self.get_logger().error(f"เกิดข้อผิดพลาดในการตรวจสอบ {file_name}: {e}")
                    else:
                        # ถ้าไม่มีไฟล์ในปลายทาง ให้ส่งไฟล์ใหม่
                        self.get_logger().info(f"ไฟล์ {file_name} หายไปจากปลายทาง กำลังอัปโหลด...")
                        sftp.put(local_file_path, remote_file_path)

                self.get_logger().info("การตรวจสอบไฟล์เสร็จสิ้น")

            except Exception as e:
                self.get_logger().error(f"เกิดข้อผิดพลาด: {e}")

            finally:
                # ปิดการเชื่อมต่อ SFTP อย่างปลอดภัย
                if sftp:
                    sftp.close()
                if transport:
                    transport.close()

            # รอและตรวจสอบใหม่ทุก `retry_interval` วินาที
            self.get_logger().info(f"จะตรวจสอบอีกครั้งใน {retry_interval} วินาที...")
            rclpy.spin_once(self, timeout_sec=retry_interval)

def main(args=None):
    rclpy.init(args=args)

    # สร้างและรัน ROS 2 node
    file_sender_node = FileSenderNode()
    
    # เรียกใช้ฟังก์ชันตรวจสอบและส่งไฟล์
    file_sender_node.monitor_and_resend_files()
    
    rclpy.spin(file_sender_node)

    # เมื่อการใช้งานเสร็จสิ้น
    file_sender_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()