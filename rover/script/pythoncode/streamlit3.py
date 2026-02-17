import streamlit as st
# from streamlit_folium import st_folium
import pandas as pd
import pydeck as pdk
import streamlit.components.v1 as components
# import folium
import json
from io import BytesIO
from PIL import Image
import geemap
import ee

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

st.set_page_config(layout="wide")

ee.Authenticate()
ee.Initialize(project='ee-khwanchai00gg')

# Create a list of coordinates representing the walking path
class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscriptions

        # Subscriptions for image topics
        self.subscription_0 = self.create_subscription(
            CompressedImage,
            '/rover/camera/usb_cam_0/image_compressed',
            self.image_callback_0,
            qos_profile
        )
        self.subscription_1 = self.create_subscription(
            CompressedImage,
            '/rover/camera/usb_cam_1/image_compressed',
            self.image_callback_1,
            qos_profile
        )

        self.motor_sub = self.create_subscription(
            String,
            '/motor_msg/rawdata',
            self.motor_callback,
            qos_profile)
        self.last_motor_callback_time = self.get_clock().now()
        self.temp_sub = self.create_subscription(
            String,
            '/temp/rawdata',
            self.temp_callback,
            qos_profile)
        self.last_temp_callback_time = self.get_clock().now()
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            qos_profile)
        self.last_diagnostics_callback_time = self.get_clock().now()
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/rawdata',
            self.imu_callback,
            qos_profile)
        self.last_imu_callback_time = self.get_clock().now()
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/rawdata',
            self.gps_callback,
            qos_profile)
        self.last_gps_callback_time = self.get_clock().now()
        self.gps_sub = self.create_subscription(
            Float64MultiArray,
            '/my_cmd',
            self.my_cmd_callback,
            qos_profile)
        self.last_my_cmd_callback_time = self.get_clock().now()

        self.last_time = self.get_clock().now()

        # Set the initial map viewport location
        self.latitude = 37.7749
        self.longitude = -122.4194
        self.icon_url = "https://upload.wikimedia.org/wikipedia/commons/e/ec/RedDot.svg"
        # Set up the layout using columns and rows
        self.setup_layout()

    def setup_layout(self):
        # Create layout with two columns: left and right
        col1, col2 = st.columns([1, 1])  # You can adjust the proportions of columns

        # Left column with 3 rows (tables)
        with col1:
            # st.write("### Left Section (Tables)")

            # Row 1: Table 1
            with st.container():
                self.table_placeholder_12 = st.empty()

            # Row 2
            with st.container():
                st.write("Temp Sensor")
                # st.markdown(self.data1.to_html(index=False), unsafe_allow_html=True)
                col_img3, col_img4 = st.columns(2)
                with col_img3:
                    self.table_placeholder_10 = st.empty()
                with col_img4:
                    self.table_placeholder_11 = st.empty()
            
            # Row 3: Table 2
            with st.container():
                st.write("Motor Feedback")
                # st.markdown(self.data2.to_html(index=False), unsafe_allow_html=True)
                self.table_placeholder_2 = st.empty()

            # Row 4: Table 3
            with st.container():
                st.write("IMU Sensor")
                # st.markdown(self.data3.to_html(index=False), unsafe_allow_html=True)
                self.table_placeholder_3 = st.empty()

        # Right column with 2 rows
        with col2:
            # st.write("### Right Section")

            # Row 1: Two columns for images
            with st.container():
                # st.write("Camera")
                col_img1, col_img2 = st.columns(2)
                with col_img1:
                    self.image_placeholder_1 = st.empty()
                    # st.image("https://via.placeholder.com/150", caption="Camera 0")
                    self.image_placeholder_1.image("https://via.placeholder.com/150", caption="Camera 0")
                with col_img2:
                    self.image_placeholder_2 = st.empty()
                    # st.image("https://via.placeholder.com/150", caption="Camera 1")
                    self.image_placeholder_2.image("https://via.placeholder.com/150", caption="Camera 1")

            # Row 2: Map
            with st.container():
                # st.write("Map from gps")
                # Create and display map using pydeck
                self.map_placeholder = st.empty()
    
    def my_cmd_callback(self, msg):
        try:
            my_cmd = ["N/A"] * 6
            my_cmd[0] = msg.data[0]
            my_cmd[1] = msg.data[1]
            my_cmd[2] = msg.data[2]
            my_cmd[3] = msg.data[3]
            my_cmd[4] = msg.data[4]
            my_cmd[5] = msg.data[5]
            self.last_my_cmd_callback_time = self.get_clock().now()

            if my_cmd[2] == 0.0:
                mode_text = "Exposit"
            elif my_cmd[2] == 1.0:
                mode_text = "Ankerman"
            elif my_cmd[2] == 2.0:
                mode_text = "Donut"
            elif my_cmd[2] == 3.0:
                mode_text = "ID"
            else:
                mode_text = "NaN"

            self.data12 = pd.DataFrame({
                'Linear': [my_cmd[0]],
                'Angular': [my_cmd[1]],
                'Mode': [mode_text],
                'Motor On': [my_cmd[3]],
                'Motor Off': [my_cmd[4]],
                'ID Motor': [int(my_cmd[5])],
            })
        except:
            self.data12 = pd.DataFrame({
                'Linear': ['N/A'],
                'Angular': ['N/A'],
                'Mode': ['N/A'],
                'Motor On': ['N/A'],
                'Motor Off': ['N/A'],
                'ID Motor': ['N/A'],
            })
            pass
        t12 = self.data12.to_html(index=False, border=0)
        # t12 = t12.replace('<table ', '<table style="text-align: center;" ')
        t12 = t12.replace('<table ', '<table style="text-align: center; width: 70%; table-layout: fixed;" ') 
        t12 = t12.replace('<th>', '<th style="text-align: center;">')
        t12 = t12.replace('<td>', '<td style="text-align: center;">')
        self.table_placeholder_12.markdown(t12, unsafe_allow_html=True)
    
    def gps_callback(self, msg):
        try:
            gps = ["N/A"] * 3
            gps[0] = msg.latitude
            gps[1] = msg.longitude
            gps[2] = msg.altitude
            self.last_gps_callback_time = self.get_clock().now()

            try:
                # Update latitude and longitude with valid GPS data
                self.latitude = float(gps[0])
                self.longitude = float(gps[1])
            except ValueError:
                # Use fallback coordinates if parsing fails
                self.latitude = 37.7749
                self.longitude = -122.4194
        except Exception as e:
            # Handle the case where the GPS message itself is invalid or incomplete
            print(f"Error processing GPS data: {e}")

        now = self.get_clock().now()
        if self.last_time is None or ((now - self.last_time).nanoseconds / 1e9) >= 1.0:
            self.last_time = now
            # Set the viewport location
            view_state = pdk.ViewState(
                latitude=self.latitude,
                longitude=self.longitude,
                zoom=16,
                pitch=0  # Set pitch to 0 for top-down view
            )

            # Marker data with icon
            marker_data = pd.DataFrame({
                'lat': [self.latitude],
                'lon': [self.longitude],
                'icon_data': [{
                    'url': self.icon_url,
                    'width': 128,
                    'height': 128,
                    'anchorY': 128
                }]
            })

            # Icon layer
            icon_layer = pdk.Layer(
                'IconLayer',
                marker_data,
                get_position='[lon, lat]',
                get_icon='icon_data',
                get_size=1,  # Icon size
                size_scale=15,
                pickable=True
            )

            # Use Mapbox Satellite Style URL for satellite imagery
            map_style = "mapbox://styles/mapbox/satellite-streets-v11"

            # Create and render the deck with Mapbox satellite style
            r = pdk.Deck(
                layers=[icon_layer],  # Icon layer should be placed over the map
                initial_view_state=view_state,
                map_style=map_style,  # Apply the satellite style
                
            )

            # Display the map in Streamlit
            self.map_placeholder.pydeck_chart(r)

    def imu_callback(self, msg):
        try:
            imu_point = 4
            imu = ["N/A"] * 10
            imu[0] = round(msg.orientation.x, imu_point)
            imu[1] = round(msg.orientation.y, imu_point)
            imu[2] = round(msg.orientation.z, imu_point)
            imu[3] = round(msg.orientation.w, imu_point)
            imu[4] = round(msg.angular_velocity.x, imu_point)
            imu[5] = round(msg.angular_velocity.y, imu_point)
            imu[6] = round(msg.angular_velocity.z, imu_point)
            imu[7] = round(msg.linear_acceleration.x, imu_point)
            imu[8] = round(msg.linear_acceleration.y, imu_point)
            imu[9] = round(msg.linear_acceleration.z, imu_point)
            self.last_imu_callback_time = self.get_clock().now()
            self.data3 = pd.DataFrame({
                    'Name': ['Orientation', 'Angular Velocity', 'Linear Acceleration'],
                    'x': [imu[0], imu[4], imu[7]],
                    'y': [imu[1], imu[5], imu[8]],
                    'z': [imu[2], imu[6], imu[9]],
                    'w': [imu[3], "N/A", "N/A"],
                })
        except:
            self.data3 = pd.DataFrame({
                'Name': ['Orientation', 'Angular Velocity', 'Linear Acceleration'],
                'x': ["N/A", "N/A", "N/A"],
                'y': ["N/A", "N/A", "N/A"],
                'z': ["N/A", "N/A", "N/A"],
                'w': ["N/A", "N/A", "N/A"],
            })
            pass
        t3 = self.data3.to_html(index=False, border=0)
        t3 = t3.replace('<table ', '<table style="text-align: center;" ')
        t3 = t3.replace('<th>', '<th style="text-align: center;">')
        t3 = t3.replace('<td>', '<td style="text-align: center;">')
        self.table_placeholder_3.markdown(t3, unsafe_allow_html=True)

    def temp_callback(self, msg):
        try:
            temp_point = 2
            temp_obj = json.loads(msg.data)
            for obj in temp_obj:
                if obj['name'] == 'Temp_0':
                    temp_1 = round(float(obj['temp']), temp_point)
                    humd_1 = round(float(obj['humd']), temp_point)
                elif obj['name'] == 'Temp_1':
                    temp_2 = obj['temp']
                    humd_2 = obj['humd']
            self.data11 = pd.DataFrame({
                'Temp': [temp_1],
                'Humd': [humd_1],
            })
        except:
            self.data11 = pd.DataFrame({
                'Temp': ['N/A'],
                'Humd': ['N/A'],
            })
            pass
        t11 = self.data11.to_html(index=False, border=0)
        t11 = t11.replace('<table ', '<table style="text-align: center;" ')
        t11 = t11.replace('<th>', '<th style="text-align: center;">')
        t11 = t11.replace('<td>', '<td style="text-align: center;">')
        self.table_placeholder_11.markdown(t11, unsafe_allow_html=True)
    
    def diagnostics_callback(self, msg):
        try:
            if len(msg.status[0].values) >= 4:
                cpu_temp = round((float(msg.status[0].values[0].value) + float(msg.status[0].values[1].value) + float(msg.status[0].values[2].value) + float(msg.status[0].values[3].value)) / 4, 2)
            else:
                cpu_temp = 'N/A'
                print(f"Error: Expected at least 4 values, but got {len(msg.status[0].values)}")
            self.last_diagnostics_callback_time = self.get_clock().now()
            self.data10 = pd.DataFrame({
                'Name': ['Value'],
                'CPU Temp': [cpu_temp],
            })
        except:
            self.data10 = pd.DataFrame({
                'Name': ['Value'],
                'CPU Temp': ['N/A'],
            })
            pass
        t10 = self.data10.to_html(index=False, border=0)
        t10 = t10.replace('<table ', '<table style="text-align: center;" ')
        t10 = t10.replace('<th>', '<th style="text-align: center;">')
        t10 = t10.replace('<td>', '<td style="text-align: center;">')
        self.table_placeholder_10.markdown(t10, unsafe_allow_html=True)
    
    def motor_callback(self, msg):
        try:
            motor_point = 3
            motor_obj = json.loads(msg.data)
            self.last_motor_callback_time = self.get_clock().now()
            json_obj = {
                "motor": {
                    "wheel_fr": motor_obj[0],  # robot_motor 1
                    "wheel_fl": motor_obj[1],
                    "wheel_rr": motor_obj[2],
                    "wheel_rl": motor_obj[3],
                    "str_fr": motor_obj[4],
                    "str_fl": motor_obj[5],
                    "str_rr": motor_obj[6],
                    "str_rl": motor_obj[7],
                },
            }
            # Create the DataFrame with Name column
            self.data2 = pd.DataFrame({
                'Name': ["wheel_fr", "wheel_fl", "wheel_rr", "wheel_rl", "str_fr", "str_fl", "str_rr", "str_rl"]
            })
            # Create mapping dictionaries for each attribute
            attributes = ['name', 'effort', 'position', 'tempareture', 'velocity', 'voltage', 'current', 'move']
            mapped_data = {
                attr: self.data2['Name'].map(
                    lambda x: round(json_obj["motor"].get(x, {}).get(attr, None), motor_point)
                    if isinstance(json_obj["motor"].get(x, {}).get(attr, None), (int, float))
                    else json_obj["motor"].get(x, {}).get(attr, None)
                )
                for attr in attributes
            }
            # Create a DataFrame with the mapped attributes
            self.data2 = pd.DataFrame(mapped_data)
        except:
            self.data2 = pd.DataFrame({
                'Name': ["wheel_fr", "wheel_fl", "wheel_rr", "wheel_rl", "str_fr", "str_fl", "str_rr", "str_rl"],
                'ID': ['N/A','N/A','N/A','N/A','N/A','N/A','N/A','N/A'],
                'Effort': ['N/A','N/A','N/A','N/A','N/A','N/A','N/A','N/A'],
                'Position': ['N/A','N/A','N/A','N/A','N/A','N/A','N/A','N/A'],
                'Tempareture': ['N/A','N/A','N/A','N/A','N/A','N/A','N/A','N/A'],
                'Velocity': ['N/A','N/A','N/A','N/A','N/A','N/A','N/A','N/A'],
                'Voltage': ['N/A','N/A','N/A','N/A','N/A','N/A','N/A','N/A'],
                'Current': ['N/A','N/A','N/A','N/A','N/A','N/A','N/A','N/A'],
                'Move': ['N/A','N/A','N/A','N/A','N/A','N/A','N/A','N/A'],
            })
            pass
        t2 = self.data2.to_html(index=False, border=0)
        t2 = t2.replace('<table ', '<table style="text-align: center;" ')
        t2 = t2.replace('<th>', '<th style="text-align: center;">')
        t2 = t2.replace('<td>', '<td style="text-align: center;">')
        self.table_placeholder_2.markdown(t2, unsafe_allow_html=True)
    
    def image_callback_0(self, msg):
        try:
            # Create an image object from the byte data
            image = Image.open(BytesIO(msg.data))
            self.image_placeholder_1.image(image, caption="Camera 0")
        except:
            self.image_placeholder_1.image("https://via.placeholder.com/150", caption="Camera 0")
            pass

    def image_callback_1(self, msg):
        try:
            # Create an image object from the byte data
            image = Image.open(BytesIO(msg.data))
            self.image_placeholder_2.image(image, caption="Camera 1")
        except:
            self.image_placeholder_2.image("https://via.placeholder.com/150", caption="Camera 1")
            pass

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    node = GUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
