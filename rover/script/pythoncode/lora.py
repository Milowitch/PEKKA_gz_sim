#!/usr/bin/python3
import serial
import time
import threading
import json
import zlib
import base64


class StationLoRa:
    def __init__(self, serial_port='COM3', baud_rate=115200, send_rate=1.5, target_addr=0, target_ch=18, lost_connect_time=5.0):
        self.serial_port_name = serial_port
        self.baud_rate = baud_rate
        self.send_rate = send_rate
        self.target_addr = target_addr
        self.target_ch = target_ch
        self.lost_connect_time = lost_connect_time

        self.serial_port = None
        self.send_data = ''
        self.serial_read_error_count = 0
        self.serial_read_count = 0

        self.connect_serial_port()

        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

        self.write_thread = threading.Thread(target=self.write_to_serial)
        self.write_thread.daemon = True
        self.write_thread.start()

    def connect_serial_port(self):
        """พยายามเชื่อมต่อกับ Serial Port"""
        while self.serial_port is None:
            try:
                self.serial_port = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1)
                # print(f'Successfully connected to {self.serial_port_name}')
            except Exception as e:
                # print(f'Error connecting to serial port: {e}')
                # print('Retrying in 5 seconds...')
                time.sleep(5)

    def read_from_serial(self):
        """อ่านข้อมูลจาก Serial Port"""
        while True:
            if self.serial_port is None or not self.serial_port.is_open:
                # print('Serial port not connected. Attempting to reconnect...')
                self.connect_serial_port()
                continue

            try:
                if self.serial_port.in_waiting > 0:
                    # อ่านข้อมูลทั้งหมดในรอบเดียว
                    buffer = self.serial_port.read(self.serial_port.in_waiting)  # อ่านข้อมูลทั้งหมดที่มีอยู่ในบัฟเฟอร์
                    
                    decoded_data = base64.b64decode(buffer[:-2])

                    # Step 3: Decompress the data
                    decompressed_data = zlib.decompress(decoded_data).decode()

                    # Step 4: Deserialize JSON
                    received_json = json.loads(decompressed_data)

                    # Display received 00       
                    # Log information0 
                    print("Received Data:", received_json)

                    size = len(decoded_data)
                    self.serial_read_count += 1
                    print(f'Receive size: {size} bytes | Read: {self.serial_read_count} | Errors: {self.serial_read_error_count}')

                    rssi_str = buffer[-2:]
                    rssi_decimal = -int(rssi_str, 16)
                    print(f'RSSI : {rssi_decimal}')
            except Exception as e:
                print(f'Serial read error: {e}')

                if self.serial_port:
                    self.serial_port.close()
                    self.serial_port = None
                self.serial_read_count += 1
                self.serial_read_error_count += 1
                continue
            time.sleep(0.001)

    def write_to_serial(self):
        """เขียนข้อมูลไปยัง Serial Port"""
        while True:
            try:
                if self.send_data != '':
                    message = self.send_data.strip()
                    # print(f'Sending Data: {message}')
                    size1 = len(message)

                    if self.serial_port is not None and self.serial_port.is_open:
                        compressed_data = zlib.compress(message.encode(), level=9)
                        # print(f'compress size: {sys.getsizeof(compressed_data) * 8}')

                        encoded_data = base64.b64encode(compressed_data)

                        combined = self.target_addr.to_bytes(2, byteorder='big') + self.target_ch.to_bytes(1, byteorder='big') + b'\r\n' + encoded_data
                        size2 = len(combined)

                        # Step 4: Send the compressed data
                        self.serial_port.write(combined)
                        # self.serial_port.write(b'\r\n')
                        self.serial_port.flush()
                        print(f'Sending Data: {message} Send size: {size1}-->{size2} bytes')
            except Exception as e:
                # print(f'Serial write error: {e}')
                if self.serial_port:
                    self.serial_port.close()
                    self.serial_port = None
                continue

            time.sleep(self.send_rate)

    def set_send_data(self, data):
        """ตั้งค่าข้อมูลที่จะส่งไปยัง Serial Port"""
        self.send_data = data


def main():
    # ตั้งค่าพอร์ตและอัตรา baud rate
    send_rate = 1.5

    station_lora = StationLoRa(serial_port='COM3', send_rate=send_rate, baud_rate=115200)


    try:
        number = 0
        while True:
            # ตัวอย่างการส่งข้อมูล
            data_to_send = json.dumps({"message": f"Hello, LoRa! {number}"})
            number+=1
            station_lora.set_send_data(data_to_send)
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Shutting down...")
        if station_lora.serial_port:
            station_lora.serial_port.close()


if __name__ == '__main__':
    main()                                                                                                                                                                                                                                                                               