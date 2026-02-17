#!/usr/bin/python3
import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import json

import folium
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl, QTimer

class MapWindow(QMainWindow):
    def __init__(self, map_path):
        super().__init__()

        self.latitude = 51.5074  # Initial latitude (e.g., London)
        self.longitude = -0.1278  # Initial longitude (e.g., London)

        # Create a folium map and display it in QWebEngineView
        self.map_widget = QWebEngineView()
        self.map_path = map_path
        self.create_map(self.latitude, self.longitude)

        # Set layout
        layout = QVBoxLayout()
        layout.addWidget(self.map_widget)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def create_map(self, latitude, longitude):
        # Generate map with folium
        my_map = folium.Map(location=[latitude, longitude], zoom_start=12)
        folium.Marker([latitude, longitude], popup="Your Location").add_to(my_map)
        my_map.save(self.map_path)  # Save the map to the specified path

        # Convert file path to QUrl and load it in QWebEngineView
        url = QUrl.fromLocalFile(self.map_path)
        self.map_widget.setUrl(url)

    def update_map(self, latitude, longitude):
        # Update the map with new latitude and longitude
        self.latitude = latitude
        self.longitude = longitude
        self.create_map(self.latitude, self.longitude)

class GUINode(Node):
    def __init__(self, map_window):
        super().__init__('gui_node')

        # Store reference to the map window
        self.map_window = map_window

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscriptions
        self.station_lora_sub = self.create_subscription(
            String,
            'station_LoRa/rawdata',
            self.listener_callback,
            qos_profile
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/rawdata',
            self.gps_callback,
            qos_profile
        )

        self.latitude = None
        self.longitude = None

    def listener_callback(self, msg):
        json_obj = json.loads(msg.data)
        # Process the json_obj here

    def gps_callback(self, msg):
        # Update latitude and longitude based on GPS data
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.get_logger().info(f"Received GPS data: Latitude {self.latitude}, Longitude {self.longitude}")

        # Update the map in the GUI
        self.map_window.update_map(self.latitude, self.longitude)

def ros_spin(node):
    """Run the ROS event loop with non-blocking spinning."""
    rclpy.spin_once(node, timeout_sec=0.1)

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Initialize Qt application
    app = QApplication(sys.argv)

    # Set the path for the map file
    map_path = "/home/station/my_map3.html"  # Path to save the map

    # Create the map window
    map_window = MapWindow(map_path)
    map_window.show()

    # Create the ROS 2 node and pass the map window to it
    node = GUINode(map_window)

    # Create a QTimer to periodically run the ROS spin
    timer = QTimer()
    timer.timeout.connect(lambda: ros_spin(node))
    timer.start(100)  # Run the ROS spin every 100 ms

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
