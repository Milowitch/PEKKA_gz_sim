import tkinter as tk
from tkinter import ttk
import threading
import random
from PIL import Image, ImageTk
import time

class Robot_Gui:
    def __init__(self):
        self.cpu_temp = 'N/A'
        self.temp_1 = 'N/A'
        self.temp_2 = 'N/A'
        self.humd_1 = 'N/A'
        self.humd_2 = 'N/A'
        self.motor = ['N/A' for _ in range(8)]
        
        # Initialize GPS-related variables
        self.current_marker_index = 0
        self.path_pixel_coordinates = []

        self.gui_thread = threading.Thread(target=self.setup_gui)
        self.gui_thread.start()

    def get_status(self):
        status = {
            "temps": {
                "rct": self.cpu_temp,
                "rt1": self.temp_1,
                "rh1": self.humd_1,
                "rt2": self.temp_2,
                "rh2": self.humd_2,
            },
            "motor": {
                "wheel_fr": self.motor[0],
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

    def update_gui(self):
        status = self.get_status()

        for temp_key, temp_value in status['temps'].items():
            current_id = self.table1.item(temp_key, 'values')[0]
            formatted_temp_value = f"{temp_value:.2f}" if isinstance(temp_value, float) else temp_value
            self.table1.item(temp_key, values=(current_id, formatted_temp_value))

        for motor_key, motor_data in status['motor'].items():
            current_name = self.table2.item(motor_key, 'values')[0]
            self.table2.item(motor_key, values=(current_name, motor_data, 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))

    def simulate_data_update(self):
        self.cpu_temp = round(random.uniform(40, 70), 2)
        self.temp_1 = round(random.uniform(20, 30), 2)
        self.temp_2 = round(random.uniform(20, 30), 2)
        self.humd_1 = round(random.uniform(30, 60), 2)
        self.humd_2 = round(random.uniform(30, 60), 2)
        self.motor = [round(random.uniform(0, 100), 2) for _ in range(8)]

    def lat_lon_to_pixel(self, lat, lon):
        x = (lon - self.top_left_lon) * self.lon_scale
        y = (self.top_left_lat - lat) * self.lat_scale
        return int(x), int(y)

    def interpolate_points(self, start, end, num_points):
        lat1, lon1 = start
        lat2, lon2 = end
        lat_step = (lat2 - lat1) / num_points
        lon_step = (lon2 - lon1) / num_points
        return [(lat1 + i * lat_step, lon1 + i * lon_step) for i in range(num_points + 1)]

    def move_marker(self):
        if self.current_marker_index < len(self.path_pixel_coordinates):
            coord = self.path_pixel_coordinates[self.current_marker_index]
            self.gps_canvas.coords(self.marker, coord[0], coord[1])
            self.current_marker_index += 1
            self.root.after(100, self.move_marker)  # Adjust the delay as needed
        else:
            self.current_marker_index = 0  # Reset to start again or stop

    def setup_gui(self):
        try:
            total_width = 1600
            self.root = tk.Tk()
            self.root.title("Robot Status")
            self.root.geometry(f"{total_width}x550")

            main_frame = tk.Frame(self.root)
            main_frame.pack(fill=tk.BOTH, expand=True)

            left_frame = tk.Frame(main_frame, width=int(total_width * 0.6))
            left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

            right_frame = tk.Frame(main_frame, width=int(total_width * 0.4), bg='gray')
            right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

            frame1 = tk.Frame(left_frame)
            frame1.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=5, fill=tk.BOTH, expand=True)

            frame2 = tk.Frame(left_frame)
            frame2.pack(side=tk.TOP, anchor=tk.W, padx=5, pady=5, fill=tk.BOTH, expand=True)

            self.table1 = ttk.Treeview(frame1, columns=("Name", "Value"), show='headings')
            self.table1.heading("Name", text="Name")
            self.table1.heading("Value", text="Value")

            num_columns1 = 2
            column_width1 = int(total_width * 0.6) // num_columns1

            self.table1.column("Name", anchor=tk.CENTER, stretch=True, width=column_width1)
            self.table1.column("Value", anchor=tk.CENTER, stretch=True, width=column_width1)

            self.table1.insert("", "end", "rct", values=("CPU Temp", 'N/A'))
            self.table1.insert("", "end", "rt1", values=("Temp 1", 'N/A'))
            self.table1.insert("", "end", "rh1", values=("Humd 1", 'N/A'))
            self.table1.insert("", "end", "rt2", values=("Temp 2", 'N/A'))
            self.table1.insert("", "end", "rh2", values=("Humd 2", 'N/A'))

            self.table1.pack(fill=tk.BOTH, expand=True)

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

            num_columns2 = 9
            column_width2 = int(total_width * 0.6) // num_columns2

            for column in self.table2["columns"]:
                self.table2.column(column, anchor=tk.CENTER, stretch=True, width=column_width2)

            self.table2.insert("", "end", "wheel_fr", values=("Wheel fr", 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))
            self.table2.insert("", "end", "wheel_fl", values=("Wheel fl", 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))
            self.table2.insert("", "end", "wheel_rr", values=("Wheel rr", 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))
            self.table2.insert("", "end", "wheel_rl", values=("Wheel rl", 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))
            self.table2.insert("", "end", "str_fr", values=("Steering fr", 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))
            self.table2.insert("", "end", "str_fl", values=("Steering fl", 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))
            self.table2.insert("", "end", "str_rr", values=("Steering rr", 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))
            self.table2.insert("", "end", "str_rl", values=("Steering rl", 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'))

            self.table2.pack(fill=tk.BOTH, expand=True)

            refresh_button = tk.Button(left_frame, text="Refresh", command=self.refresh_gui)
            refresh_button.pack(pady=10)

            # Split the right frame into two sections: Camera (60%) and GPS (40%)
            camera_frame = tk.Frame(right_frame, bg='gray', height=800)  # 60% of 550px is approximately 330px
            camera_frame.pack(fill=tk.X, expand=True, padx=10, pady=5)

            gps_frame = tk.Frame(right_frame, bg='gray', height=360)  # 40% of 550px is approximately 220px
            gps_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

            # Camera section
            camera1_label = tk.Label(camera_frame, text="Camera 1", bg='darkgray', fg='white')
            camera1_label.pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=5)

            camera2_label = tk.Label(camera_frame, text="Camera 2", bg='darkgray', fg='white')
            camera2_label.pack(side=tk.RIGHT, expand=True, fill=tk.BOTH, padx=5)

            # GPS Section
            # Create a canvas in gps_frame
            self.gps_canvas = tk.Canvas(gps_frame, width=550, height=300)
            self.gps_canvas.pack(fill=tk.BOTH, expand=True)

            # Load map image
            map_image = Image.open("/home/mekkkk/ros2_ws/src/rover_ros2/rover/script/map.png")  # Replace with your map image path
            map_image = map_image.resize((550, 300), Image.LANCZOS)
            self.map_photo = ImageTk.PhotoImage(map_image)

            # Place the map image on the canvas
            self.gps_canvas.create_image(0, 0, anchor=tk.NW, image=self.map_photo)

            # Load marker image
            marker_image = Image.open("/home/mekkkk/ros2_ws/src/rover_ros2/rover/script/vecteezy_red-circle-png-red-dot-icon_16314339.png")  # Replace with your marker image path
            marker_image = marker_image.resize((20, 20), Image.LANCZOS)
            self.marker_photo = ImageTk.PhotoImage(marker_image)

            # Create an image item on the canvas for the marker
            self.marker = self.gps_canvas.create_image(100, 100, image=self.marker_photo)

            # Define the bounds of the map (latitude and longitude of the top-left and bottom-right corners)
            self.top_left_lat = 13.903513
            self.top_left_lon = 100.529644
            self.bottom_right_lat = 13.900702
            self.bottom_right_lon = 100.533367

            # Calculate the scaling factors
            map_width = 550  # Width of the map image in pixels
            map_height = 300  # Height of the map image in pixels
            self.lat_scale = map_height / (self.top_left_lat - self.bottom_right_lat)
            self.lon_scale = map_width / (self.bottom_right_lon - self.top_left_lon)

            # Create a list of coordinates representing the walking path (latitude, longitude)
            path_coordinates = [
                (13.9018, 100.5317),
                (13.9020, 100.5320),
                (13.9022, 100.5323),
                (13.9024, 100.5326),
                (13.901728, 100.532166)
            ]

            # Interpolate additional points between the existing coordinates
            num_interpolated_points = 10  # Number of points to interpolate between each pair of coordinates
            detailed_path_coordinates = []
            for i in range(len(path_coordinates) - 1):
                start = path_coordinates[i]
                end = path_coordinates[i + 1]
                detailed_path_coordinates.extend(self.interpolate_points(start, end, num_interpolated_points))

            # Convert the detailed path coordinates to pixel coordinates
            self.path_pixel_coordinates = [self.lat_lon_to_pixel(lat, lon) for lat, lon in detailed_path_coordinates]

            # Start index for movement
            self.current_marker_index = 0

            # Start the movement
            self.move_marker()

            def periodic_update():
                self.simulate_data_update()
                self.update_gui()
                self.root.after(1000, periodic_update)

            periodic_update()
            self.root.mainloop()

        except Exception as e:
            print(f"Error in setup_gui: {e}")

    def refresh_gui(self):
        self.temp_1 = 'N/A'
        self.temp_2 = 'N/A'
        self.humd_1 = 'N/A'
        self.humd_2 = 'N/A'
        self.motor = ['N/A' for _ in range(8)]
        self.update_gui()

robot_gui = Robot_Gui()
