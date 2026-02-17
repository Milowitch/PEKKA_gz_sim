import sys
import os
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QVBoxLayout, QSplitter, QTableWidget, QTableWidgetItem, QGridLayout , QHBoxLayout
from PyQt5.QtCore import Qt, QTimer, QUrl
from PyQt5.QtGui import QFont, QKeyEvent
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWidgets import QPushButton

import random

MAX_CID = 8  # Define the maximum number of motors

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("GUI with Real-Time Data")
        
        # Set the application to normal windowed mode
        self.setGeometry(100, 100, 1920, 1080)  # Full HD resolution

        self.setFont(QFont("Arial", 10))

        # Central widget
        central_widget = QWidget()
        central_widget.setStyleSheet("background-color: #ffffff;")  # White background
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QVBoxLayout()

        # Splitter to divide left and right sections
        splitter = QSplitter(Qt.Horizontal)
        
        # Left side (60%)
        left_widget = QWidget()
        left_widget.setStyleSheet("background-color: #f0f0f0;")  # Light background
        left_layout = QVBoxLayout()
        left_layout.setContentsMargins(10, 10, 10, 10)

        # Reset button
        
        reset_button = QPushButton("Reset GUI")
        reset_button.setFixedHeight(40)
        reset_button.setStyleSheet("""
            background-color: #FF6347;
            color: white;
            font-weight: bold;
            border-radius: 5px;
        """)
        reset_button.clicked.connect(self.reset_gui)
        left_layout.addWidget(reset_button)
        
        # Motor table with attributes
        self.motor_table = self.create_motor_table("Motor")
        self.motor_table.setFixedHeight(int(1080 * 0.375))  # 35% of the screen height
        left_layout.addWidget(self.motor_table)
        
        # IMU table with simulated data
        self.imu_table = self.create_table("IMU", 3, 3)
        left_layout.addWidget(self.imu_table)
        
        # OBC table with specific rows
        self.obc_table = self.create_table("OBC", 5, 1)  # 5 rows for cputemp, Temp1, Humd1, Temp2, Humd2
        left_layout.addWidget(self.obc_table)

        left_widget.setLayout(left_layout)
        
        # Right side (40%)
        right_widget = QWidget()
        right_widget.setStyleSheet("background-color: #f0f0f0;")  # Light background
        right_layout = QVBoxLayout()
        right_layout.setContentsMargins(10, 10, 10, 10)
        
        # Camera view placeholder (60% height)
        camera_layout = QHBoxLayout()  # Horizontal layout for splitting into left and right

        left_camera_label = QLabel("Camera1")
        left_camera_label.setStyleSheet("""
            background-color: #e0e0e0;
            color: black;
            border-radius: 0px;
            padding: 0px;
            font-size: 6px;
        """)
        left_camera_label.setAlignment(Qt.AlignCenter)

        right_camera_label = QLabel("Camera2")
        right_camera_label.setStyleSheet("""
            background-color: #e0e0e0;
            color: black;
            border-radius: 0px;
            padding: 0px;
            font-size: 6px;
        """)
        right_camera_label.setAlignment(Qt.AlignCenter)

        camera_layout.addWidget(left_camera_label)
        camera_layout.addWidget(right_camera_label)

        camera_container = QWidget()
        camera_container.setLayout(camera_layout)
        camera_container.setFixedHeight(int(1080 * 0.6))  # 60% of the screen height
        right_layout.addWidget(camera_container)

        
        # GPS Map (40% height)
        self.map_view = QWebEngineView()
        self.map_view.setFixedHeight(int(1080 * 0.4))  # 40% of the screen height
        right_layout.addWidget(self.map_view)

        def update_map_periodically():
            lat = 13.9018
            lon = 100.5317
            self.update_map(lat, lon)
            QTimer.singleShot(3000, update_map_periodically)

        QTimer.singleShot(3000, update_map_periodically)

        right_widget.setLayout(right_layout)
        
        # Add widgets to the splitter
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        
        splitter.setSizes([60, 40])  # 60% for left, 40% for right

        main_layout.addWidget(splitter)
        central_widget.setLayout(main_layout)

        # Timer for updating IMU and GPS data
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(1000)  # Update every second

    def create_motor_table(self, title):
        table_widget = QWidget()
        grid_layout = QGridLayout()
        grid_layout.setContentsMargins(10, 10, 10, 10)
        
        label = QLabel(title)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("""
            background-color: #4CAF50;
            color: white;
            font-weight: bold;
            font-size: 12px;
            padding: 10px;
            border-radius: 5px;
        """)
        grid_layout.addWidget(label, 0, 0, 1, 8)  # Span 7 columns
        
        table = QTableWidget(MAX_CID, 8)
        table.setHorizontalHeaderLabels(["Name", "ID", "Effort", "Position", "Temperature", "Velocity", "Voltage", "Current", "Move"])
        table.setStyleSheet("""
            background-color: #ffffff;
            color: black;
            border-radius: 5px;
        """)
        table.horizontalHeader().setStyleSheet("""
            background-color: #e0e0e0;
            color: black;
            font-weight: bold;
        """)
        table.verticalHeader().setStyleSheet("""
            background-color: #e0e0e0;
            color: black;
            font-weight: bold;
        """)
        table.setAlternatingRowColors(True)
        table.setStyleSheet("""
            QTableWidget::item {
                background-color: #ffffff;
                color: black;
                border: 1px solid #cccccc;
            }
            QTableWidget::item:selected {
                background-color: #dddddd;
            }
        """)
        grid_layout.addWidget(table, 1, 0, 1, 8)  # Span 7 columns
        
        table_widget.setLayout(grid_layout)
        return table_widget

    def create_table(self, title, rows, columns):
        table_widget = QWidget()
        grid_layout = QGridLayout()
        grid_layout.setContentsMargins(10, 10, 10, 10)
        
        label = QLabel(title)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("""
            background-color: #4CAF50;
            color: white;
            font-weight: bold;
            font-size: 14px;
            padding: 10px;
            border-radius: 5px;
        """)
        grid_layout.addWidget(label, 0, 0, 1, columns)  # Span columns
        
        table = QTableWidget(rows, columns)
        if title == "IMU":
            table.setHorizontalHeaderLabels(["X", "Y", "Z"])  # For IMU: 3 columns for X, Y, Z data
            table.setVerticalHeaderLabels(["Acceleration", "Gyroscope", "Angles"])  # IMU specific rows
        elif title == "OBC":
            table.setHorizontalHeaderLabels(["Value1"])  # For OBC: 4 columns
            table.setVerticalHeaderLabels(["cputemp", "Temp1", "Humd1", "Temp2", "Humd2"])  # OBC specific rows
        else:
            table.setHorizontalHeaderLabels(["Column1", "Column2", "Column3", "Column4"])  # Example
            table.setVerticalHeaderLabels(["Row1", "Row2", "Row3"])  # Default rows

        table.setStyleSheet("""
            background-color: #ffffff;
            color: black;
            border-radius: 5px;
        """)
        table.horizontalHeader().setStyleSheet("""
            background-color: #e0e0e0;
            color: black;
            font-weight: bold;
        """)
        table.verticalHeader().setStyleSheet("""
            background-color: #e0e0e0;
            color: black;
            font-weight: bold;
        """)
        table.setAlternatingRowColors(True)
        table.setStyleSheet("""
            QTableWidget::item {
                background-color: #ffffff;
                color: black;
                border: 1px solid #cccccc;
            }
            QTableWidget::item:selected {
                background-color: #dddddd;
            }
        """)
        grid_layout.addWidget(table, 1, 0, 1, columns)  # Span columns
        
        table_widget.setLayout(grid_layout)
        return table_widget

    def update_data(self):
        # Simulate IMU data
        acc_x = random.uniform(-1, 1) * 9.81
        acc_y = random.uniform(-1, 1) * 9.81
        acc_z = random.uniform(-1, 1) * 9.81

        gyro_x = random.uniform(-250, 250)
        gyro_y = random.uniform(-250, 250)
        gyro_z = random.uniform(-250, 250)

        angle_x = random.uniform(-180, 180)
        angle_y = random.uniform(-180, 180)
        angle_z = random.uniform(-180, 180)

        # Update IMU table with this data
        imu_table = self.imu_table.findChild(QTableWidget)
        imu_table.setItem(0, 0, QTableWidgetItem(f"{acc_x:.2f}"))
        imu_table.setItem(0, 1, QTableWidgetItem(f"{acc_y:.2f}"))
        imu_table.setItem(0, 2, QTableWidgetItem(f"{acc_z:.2f}"))
        imu_table.setItem(1, 0, QTableWidgetItem(f"{gyro_x:.2f}"))
        imu_table.setItem(1, 1, QTableWidgetItem(f"{gyro_y:.2f}"))
        imu_table.setItem(1, 2, QTableWidgetItem(f"{gyro_z:.2f}"))
        imu_table.setItem(2, 0, QTableWidgetItem(f"{angle_x:.2f}"))
        imu_table.setItem(2, 1, QTableWidgetItem(f"{angle_y:.2f}"))
        imu_table.setItem(2, 2, QTableWidgetItem(f"{angle_z:.2f}"))

        # Update Motor table with simulated data
        motor_table = self.motor_table.findChild(QTableWidget)
        for i in range(MAX_CID):
            id = random.randint(1, 100)  # ID
            effort = random.randint(0, 100)  # Effort
            position = random.randint(0, 100)  # Position
            temperature = random.uniform(20, 100)  # Temperature
            velocity = random.randint(0, 100)  # Velocity
            voltage = random.uniform(10, 30)  # Voltage
            current = random.uniform(0, 10)  # Current

            motor_table.setItem(i, 0, QTableWidgetItem(f"{id}"))
            motor_table.setItem(i, 1, QTableWidgetItem(f"{effort}"))
            motor_table.setItem(i, 2, QTableWidgetItem(f"{position}"))
            motor_table.setItem(i, 3, QTableWidgetItem(f"{temperature:.2f}"))
            motor_table.setItem(i, 4, QTableWidgetItem(f"{velocity}"))
            motor_table.setItem(i, 5, QTableWidgetItem(f"{voltage:.2f}"))
            motor_table.setItem(i, 6, QTableWidgetItem(f"{current:.2f}"))

    def keyPressEvent(self, event: QKeyEvent):
        if event.key() == Qt.Key_T:
            self.close()  

    def reset_gui(self):
        # Reset IMU Table
        imu_table = self.imu_table.findChild(QTableWidget)
        imu_table.clearContents()

        # Reset Motor Table
        motor_table = self.motor_table.findChild(QTableWidget)
        motor_table.clearContents()

        # Reset OBC Table
        obc_table = self.obc_table.findChild(QTableWidget)
        obc_table.clearContents()

    def update_map(self, lat, lon):
        # Set the map's URL
        map_file_path = os.path.join(os.path.dirname(__file__), 'map.html')
        self.map_view.setUrl(QUrl.fromLocalFile(map_file_path))
        
        # Wait until the page is fully loaded, then run the JavaScript
        def run_js():
            js_command = f"updateMarkerPosition({lat}, {lon});"
            self.map_view.page().runJavaScript(js_command)
        
        # Connect the JavaScript execution to the loadFinished signal
        self.map_view.loadFinished.connect(run_js)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())