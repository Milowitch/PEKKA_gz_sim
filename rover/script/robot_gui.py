#!/usr/bin/python3
import rclpy
import rclpy.destroyable
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import rclpy.waitable
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
import serial
import time
import serial.tools.list_ports
import threading
import json
import sys
import tkinter as tk
from tkinter import ttk

class Robot_Gui(Node):
    def __init__(self):
        super().__init__('robot_gui')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        self.temp_sub = self.create_subscription(
            String,
            '/temp/rawdata',
            self.temp_callback,
            qos_profile)
        self.temp_sub
        self.last_temp_callback_time = self.get_clock().now()
        self.motor_sub = self.create_subscription(
            String,
            '/motor_msg/rawdata',
            self.motor_callback,
            qos_profile)
        self.motor_sub
        self.last_motor_callback_time = self.get_clock().now()
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            qos_profile)
        self.diagnostics_sub
        self.last_diagnostics_callback_time = self.get_clock().now()
        self.publisher_ = self.create_publisher(String, 'robot_LoRa/rawdata', qos_profile)
        self.data = String()
        self.lost_connect_time = 5.0
        self.cpu_temp = 'N/A'
        self.temp_1 = 'N/A'
        self.temp_2 = 'N/A'
        self.humd_1 = 'N/A'
        self.humd_2 = 'N/A'
        self.motor = ['N/A' for _ in range(8)]

        self.gui_thread = threading.Thread(target=self.setup_gui)
        self.gui_thread.start()

    def temp_callback(self, msg):
        try:
            temp_obj = json.loads(msg.data)
            for obj in temp_obj:
                if obj['name'] == 'Temp_0':
                    self.temp_1 = obj['temp']
                    self.humd_1 = obj['humd']
                elif obj['name'] == 'Temp_1':
                    self.temp_2 = obj['temp']
                    self.humd_2 = obj['humd']
            self.last_temp_callback_time = self.get_clock().now()
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON temp: {e}")

    def motor_callback(self, msg):
        try:
            motor_obj = json.loads(msg.data)
            self.motor = motor_obj
            self.last_motor_callback_time = self.get_clock().now()
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON motor: {e}")
        except Exception as e:
            print(f"Unexpected error in motor_callback: {e}")

    def diagnostics_callback(self, msg):
        try:
            if len(msg.status[0].values) >= 4:
                self.cpu_temp = round((float(msg.status[0].values[0].value) + float(msg.status[0].values[1].value) + float(msg.status[0].values[2].value) + float(msg.status[0].values[3].value)) / 4, 2)
            else:
                self.cpu_temp = -1.0
                print(f"Error: Expected at least 4 values, but got {len(msg.status[0].values)}")
            self.last_diagnostics_callback_time = self.get_clock().now()
        except ValueError:
            print(f"Error cannot convert cpu_temp to float")

    def get_status(self):
        try:
            status = {
                "temps": {
                    "rct": self.cpu_temp,  # robot_cpu_temp
                    "rt1": self.temp_1,    # robot_temp_1
                    "rh1": self.humd_1,    # robot_humd_1
                    "rt2": self.temp_2,    # robot_temp_2
                    "rh2": self.humd_2,    # robot_humd_2
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
            }
            return status
        except Exception as e:
            print(f"Error in get_status: {e}")
            return None

    def update_gui(self):
        try:
            status = self.get_status()
            if not status:
                return
            
            # Update temperature table
            for temp_key, temp_value in status['temps'].items():
                current_id = self.table1.item(temp_key, 'values')[0]
                formatted_temp_value = f"{temp_value:.2f}" if isinstance(temp_value, float) else temp_value
                self.table1.item(temp_key, values=(current_id, formatted_temp_value))

            # Update motor table
            for motor_key, motor_data in status['motor'].items():
                current_name = self.table2.item(motor_key, 'values')[0]
                
                # Check if motor_data is a dictionary
                if isinstance(motor_data, dict):
                    def format_motor_data(value):
                        return f"{value:.2f}" if isinstance(value, float) else value

                    self.table2.item(motor_key, values=(
                        current_name,
                        motor_data.get('name', 'N/A'),
                        format_motor_data(motor_data.get('effort', 'N/A')),
                        format_motor_data(motor_data.get('position', 'N/A')),
                        format_motor_data(motor_data.get('tempareture', 'N/A')),  # Ensure correct spelling here
                        format_motor_data(motor_data.get('velocity', 'N/A')),
                        format_motor_data(motor_data.get('voltage', 'N/A')),
                        format_motor_data(motor_data.get('current', 'N/A')),
                        format_motor_data(motor_data.get('move', 'N/A')),
                    ))
                else:
                    # If motor_data is not a dictionary, handle it gracefully
                    self.table2.item(motor_key, values=(current_name, 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))
            
            # Check connection statuses
            now = self.get_clock().now()
            if ((now - self.last_temp_callback_time).nanoseconds / 1e9) >= self.lost_connect_time:  # Convert to seconds
                self.temp_1 = 'N/A'
                self.temp_2 = 'N/A'
                self.humd_1 = 'N/A'
                self.humd_2 = 'N/A'
            if ((now - self.last_diagnostics_callback_time).nanoseconds / 1e9) >= self.lost_connect_time:  # Convert to seconds
                self.cpu_temp = 'N/A'
            if ((now - self.last_motor_callback_time).nanoseconds / 1e9) >= self.lost_connect_time:  # Convert to seconds
                self.motor = ['N/A' for _ in range(8)]
        
        except Exception as e:
            print(f"Error in update_gui: {e}")

    def setup_gui(self):
        try:
            total_width = 800
            self.root = tk.Tk()
            self.root.title("Robot Status")
            self.root.geometry(f"{total_width}x550")

            # Create Frame for Tables
            frame1 = tk.Frame(self.root)
            frame1.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=5, fill=tk.BOTH, expand=True)

            frame2 = tk.Frame(self.root)
            frame2.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=5, fill=tk.BOTH, expand=True)

            # Create Table 1 for Temperature
            self.table1 = ttk.Treeview(frame1, columns=("Name", "Value"), show='headings')
            self.table1.heading("Name", text="Name")
            self.table1.heading("Value", text="Value")

            # Calculate column width to fit the frame
            num_columns1 = 2  # Number of columns in the second table
            column_width1 = total_width // num_columns1  # Divide total width by the number of columns

            self.table1.column("Name", anchor=tk.CENTER, stretch=True, width=column_width1)
            self.table1.column("Value", anchor=tk.CENTER, stretch=True, width=column_width1)
            
            # Insert sample items for temperature
            self.table1.insert("", "end", "rct", values=("CPU Temp", 'N/A'))
            self.table1.insert("", "end", "rt1", values=("Temp 1", 'N/A'))
            self.table1.insert("", "end", "rh1", values=("Humd 1", 'N/A'))
            self.table1.insert("", "end", "rt2", values=("Temp 2", 'N/A'))
            self.table1.insert("", "end", "rh2", values=("Humd 2", 'N/A'))
            
            self.table1.pack(fill=tk.BOTH, expand=True)

            # Create Table 2 for Motor
            self.table2 = ttk.Treeview(frame2, columns=("Name", "ID", "Effort", "Position", "Temp", "Velocity", "Voltage", "Current", "Move"), show='headings')
            self.table2.heading("Name", text="Name")
            self.table2.heading("ID", text="ID")
            self.table2.heading("Effort", text="Effort")
            self.table2.heading("Position", text="Position")
            self.table2.heading("Temp", text="Temperature")
            self.table2.heading("Velocity", text="Velocity")
            self.table2.heading("Voltage", text="Voltage")
            self.table2.heading("Current", text="Current")
            self.table2.heading("Move", text="Move")

            # Calculate column width to fit the frame
            num_columns2 = 9  # Number of columns in the second table
            column_width2 = total_width // num_columns2  # Divide total width by the number of columns

            # Set column width to evenly distribute across the frame
            for column in self.table2["columns"]:
                self.table2.column(column, anchor=tk.CENTER, stretch=True, width=column_width2)
                
            # Insert sample items for motors
            self.table2.insert("", "end", "wheel_fr", values=("Wheel fr", 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))
            self.table2.insert("", "end", "wheel_fl", values=("Wheel fl", 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))
            self.table2.insert("", "end", "wheel_rr", values=("Wheel rr", 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))
            self.table2.insert("", "end", "wheel_rl", values=("Wheel rl", 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))
            self.table2.insert("", "end", "str_fr", values=("Steering fr", 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))
            self.table2.insert("", "end", "str_fl", values=("Steering fl", 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))
            self.table2.insert("", "end", "str_rr", values=("Steering rr", 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))
            self.table2.insert("", "end", "str_rl", values=("Steering rl", 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))
            
            self.table2.pack(fill=tk.BOTH, expand=True)

            refresh_button = tk.Button(self.root, text="Refresh", command=self.refresh_gui)
            refresh_button.pack(pady=10)

            # Update GUI periodically
            def periodic_update():
                self.update_gui()
                self.root.after(1000, periodic_update)

            periodic_update()
            self.root.mainloop()
        except Exception as e:
            print(f"Error in setup_gui: {e}")

    def refresh_gui(self):
        try:
            self.temp_1 = 'N/A'
            self.temp_2 = 'N/A'
            self.humd_1 = 'N/A'
            self.humd_2 = 'N/A'
            self.cpu_temp = 'N/A'
            self.motor = ['N/A' for _ in range(8)]
            # Recreate the GUI
            self.update_gui()
        except Exception as e:
            print(f"Error in refresh_gui: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Robot_Gui()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
