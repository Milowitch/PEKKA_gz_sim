#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String

import serial
import time
import serial.tools.list_ports
import threading
import json
import sys
import zlib
import struct
import base64

# Replace 'COM3' with the correct serial port for your setup
# SERIAL_PORT = '/dev/lora2'  # For Linux or MacOS
# BAUD_RATE = 115200

class Station_LoRa(Node):
    def __init__(self):
        super().__init__('station_lora')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        self.declare_parameter('serial_port', '/dev/lora2')  # String parameter  /dev/lora2
        self.declare_parameter('baud_rate', 115200)          # Integer parameter
        self.declare_parameter('send_rate', 2.5)            # Double parameter
        self.declare_parameter('target_addr', 0)            # Integer parameter
        self.declare_parameter('target_ch', 18)             # Integer parameter
        self.declare_parameter('lost_connect_time', 5.0)      # Double parameter
        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.send_rate = self.get_parameter('send_rate').get_parameter_value().double_value
        self.target_addr = self.get_parameter('target_addr').get_parameter_value().integer_value
        self.target_ch = self.get_parameter('target_ch').get_parameter_value().integer_value
        self.lost_connect_time = self.get_parameter('lost_connect_time').get_parameter_value().double_value

        self.station_lora_send = self.create_subscription(
            String,
            'station_lora/send',
            self.station_lora_send_callback,
            qos_profile)
        self.station_lora_send
        self.last_station_lora_send_callback_time = self.get_clock().now()

        self.publisher_ = self.create_publisher(String, 'station_lora/receive', qos_profile)
        self.data = String()
        self.serial_port = None
        # self.serial_port_name = SERIAL_PORT
        # self.baud_rate = BAUD_RATE
        # self._timer = self.create_timer(0.001, self.timer_callback)
        self.serial_read_error_count = 0
        self.serial_read_count = 0
        self.connect_serial_port()
        self.send_data = ''

        self.read_thread = threading.Thread(target=self.read_from_serial)
        self.read_thread.start()
        self.write_thread = threading.Thread(target=self.write_to_serial)
        self.write_thread.start()
    
    def station_lora_send_callback(self, msg):
        try:
            self.send_data = msg.data
            self.last_station_lora_send_callback_time = self.get_clock().now()
        except ValueError:
            print(f"Error cannot get data")
        # None
    
    def connect_serial_port(self):
        while self.serial_port is None:
            try:
                if not self.serial_port or not self.serial_port.is_open:
                    self.serial_port = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1)
                # self.get_logger().info(f'Successfully connected to {self.serial_port_name}')
            except:
                # self.get_logger().error(f'Error connecting to serial port')
                # self.get_logger().info('Retrying in 5 seconds...')
                time.sleep(5)
                pass

    def read_from_serial(self):
        while rclpy.ok():
            if self.serial_port is None or not self.serial_port.is_open:
                # self.get_logger().warn('Serial port not connected. Attempting to reconnect...')
                self.connect_serial_port()
                pass

            try:
                if self.serial_port.in_waiting > 0:
                    # อ่านข้อมูลทั้งหมดในรอบเดียว
                    buffer = self.serial_port.read(self.serial_port.in_waiting)  # อ่านข้อมูลทั้งหมดที่มีอยู่ในบัฟเฟอร์

                    decoded_data = base64.b64decode(buffer[:-2])

                    # self.get_logger().info(f'buffer : {buffer}')
                    # ถอดบีบอัดข้อมูล
                    decompressed_data = zlib.decompress(decoded_data).decode()
                    # Step 4: Deserialize the JSON string
                    received_json = json.loads(decompressed_data)
                    
                    # print("Received Data:", received_json)

                    self.data.data = json.dumps(received_json)
                    self.publisher_.publish(self.data)

                    size = sys.getsizeof(buffer)
                    self.serial_read_count += 1
                    self.get_logger().info(f'Receive size: {size} bytes read: {self.serial_read_count} error: {self.serial_read_error_count}')

                    rssi_str = buffer[-2:]
                    rssi_decimal = -int(rssi_str, 16)
                    self.get_logger().warn(f'RSSI : {rssi_decimal}')
            except serial.SerialException as e:
                # self.get_logger().error(f'Serial write error: {e}')
                # Close the serial port safely
                if self.serial_port:
                    try:
                        if self.serial_port.is_open:
                            self.serial_port.close()
                    except OSError as close_error:
                        # self.get_logger().error(f'Error closing serial port: {close_error}')
                        pass
                # Attempt to reconnect (optional, based on your use case)
                self.serial_port = None
                # self.get_logger().info('Serial port connection reset.')
            except Exception as e:
                # self.get_logger().error(f'Serial read error')
                # print(f'Serial read error')
                self.serial_read_count += 1
                self.serial_read_error_count += 1
                # self.publisher_.publish(self.data)
                self.get_logger().warn(f'Receive read: {self.serial_read_count} error: {self.serial_read_error_count}')

                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.close()
                    self.serial_port = None
                pass
            finally:
                # Optional: Delay to prevent tight looping in case of repeated failures
                threading.Event().wait(0.1)
            time.sleep(0.001)  # Avoid busy-waiting
    
    def write_to_serial(self):
        while rclpy.ok():
            try:
                if self.send_data != '':
                    message = self.send_data.strip() #+ '\r\n'
                    size1 = sys.getsizeof(message)
                    # self.get_logger().info(f'Send size: {size} bytes')
                    if self.serial_port is not None and self.serial_port.is_open:
                        # Step 2: Compress the JSON string
                        compressed_data = zlib.compress(message.encode(), level=9)

                        encoded_data = base64.b64encode(compressed_data)

                        combined = self.target_addr.to_bytes(2, byteorder='big') + self.target_ch.to_bytes(1, byteorder='big') + b'\r\n' + encoded_data

                        size2 = sys.getsizeof(encoded_data)

                        self.get_logger().info(f'Send size: {size1}-->{size2} bytes')
                        # Step 4: Send the compressed data
                        self.serial_port.write(combined)
                        # self.serial_port.write(b'\r\n')
                        self.serial_port.flush()
                        # self.get_logger().info('Data written to serial port')
                    else:
                        # self.get_logger().warn('Serial port not connected. Cannot send data.')
                        pass
            except serial.SerialException as e:
                # self.get_logger().error(f'Serial write error: {e}')
                # Close the serial port safely
                if self.serial_port:
                    try:
                        if self.serial_port.is_open:
                            self.serial_port.close()
                    except OSError as close_error:
                        # self.get_logger().error(f'Error closing serial port: {close_error}')
                        pass
                # Attempt to reconnect (optional, based on your use case)
                self.serial_port = None
                # self.get_logger().info('Serial port connection reset.')
            except Exception as e:
                # self.get_logger().error(f'Unexpected error: {e}')
                pass
            finally:
                # Optional: Delay to prevent tight looping in case of repeated failures
                threading.Event().wait(0.1)
            
            now = self.get_clock().now()
            if ((now - self.last_station_lora_send_callback_time).nanoseconds / 1e9) >= self.lost_connect_time: # Convert to seconds
                self.send_data = ''
            # print(f'size: {size} count: {len(message)}')
            time.sleep(self.send_rate)  # Sleep for 1 second between writes

def main(args=None):
    rclpy.init(args=args)
    node = Station_LoRa()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if node.serial_port:
        node.serial_port.close()

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
# AT+AllP=11,2,1,22,0,1,2,18,18,1,0,3,115200, "8N1",0   Set the spreading factor to key multi-parameter
# AT+RESTORE=0\r\n                                      Restore factory settings, 0: disabled, 1: enabled

# def list_serial_ports():
#     ports = serial.tools.list_ports.comports()
#     return [port.device for port in ports]

# class LoRa_SX1262:
#     def __init__(self, port, baud_rate):
#         try:
#             self.ser = serial.Serial(port, baud_rate, timeout=1)
#             time.sleep(2)  # Wait for the connection to establish
#             if not self.ser.is_open:
#                 raise serial.SerialException(f"Failed to open port {port}")
#         except serial.SerialException as e:
#             print(f"Error opening serial port: {e}")
#             self.ser = None

#     def send_command(self, command):
#         if self.ser and self.ser.is_open:
#             setting_command = "AT+EXIT\r\n"
#             self.ser.write(setting_command.encode())
#             time.sleep(0.1)
#             setting_command = "+++\r\n"
#             self.ser.write(setting_command.encode())
#             time.sleep(0.1)
#             command+= "\r\n"
#             self.ser.write(command.encode())
#             time.sleep(0.1)
#             setting_command = "AT+EXIT\r\n"
#             self.ser.write(setting_command.encode())
#             time.sleep(0.1)
#             # response = self.ser.read_all().decode()
#             # return response
#         else:
#             print("Serial port is not open.")
#             return None
    
#     def send_message(self, message):
#         if self.ser and self.ser.is_open:
#             self.ser.write(message.encode())
#             time.sleep(0.1)
#             print(f"Sent: {message}")
#         else:
#             print("Serial port is not open.")
#             return None
    
#     def read_from_serial(self):
#         if self.ser and self.ser.is_open:
#             data = self.ser.read_all().decode()
#             if data:
#                 return data
#             else:
#                 print("No Data.")
#                 return False
#         else:
#             print("Serial port is not open.")
#             return False

#     def set_configuration(self, spreading=7, bandwidth=0, encoding=1, RF_power=22, Network_ID=0, LBT=0, mode=1, Transmit_ch=18, Receive_ch=18, RSSI=0, DTU=0, port=3, baud=115200, parameters="8N1", Refactory=0):
#         command = f"AT+AllP={spreading},{bandwidth},{encoding},{RF_power},{Network_ID},{LBT},{mode},{Transmit_ch},{Receive_ch},{RSSI},{DTU},{port},{baud},{parameters},{Refactory}"
#         # return self.send_command(command)
#         self.send_command(command)

#     def close(self):
#         if self.ser and self.ser.is_open:
#             self.ser.close()

# List available serial ports
# available_ports = list_serial_ports()
# print("Available serial ports:", available_ports)