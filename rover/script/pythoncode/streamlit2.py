import streamlit as st
import pandas as pd
import pydeck as pdk
import time
import json
from io import BytesIO
from PIL import Image
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from math import radians, sin, cos, sqrt, atan2

st.set_page_config(layout="wide")
# Create a list of coordinates representing the walking path
class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscriptions
        self.station_lora_sub = self.create_subscription(
            String,
            '/station_lora/receive',
            self.listener_callback,
            qos_profile
        )

        # Subscriptions for image topics
        self.subscription_0 = self.create_subscription(
            CompressedImage,
            '/camera/usb_cam_0/image_compressed',
            self.image_callback_0,
            qos_profile
        )
        self.subscription_1 = self.create_subscription(
            CompressedImage,
            '/camera/usb_cam_1/image_compressed',
            self.image_callback_1,
            qos_profile
        )

        self.last_time = self.get_clock().now()
        self.json_obj = {}

        # Set the initial map viewport location
        self.latitude = 37.7749
        self.longitude = -122.4194
        self.icon_url = "https://upload.wikimedia.org/wikipedia/commons/e/ec/RedDot.svg"
        # Set up the layout using columns and rows
        self.setup_layout()
        self._timer = self.create_timer(1.5, self.timer_callback)

    def setup_layout(self):
        # Create layout with two columns: left and right
        col1, col2 = st.columns([1, 1])  # You can adjust the proportions of columns

        # Left column with 3 rows (tables)
        with col1:
            # st.write("### Left Section (Tables)")
            with st.container():
                col1_1, col2_1 = st.columns([1, 6])
                with col1_1:
                    st.write("LTE Latency")
                with col2_1:
                    self.table_placeholder_3 = st.empty()

            # Row 1: Table 1
            with st.container():
                st.write("Temp Sensor")
                # st.markdown(self.data1.to_html(index=False), unsafe_allow_html=True)
                self.table_placeholder_1 = st.empty()
            
            # Row 2: Table 2
            with st.container():
                st.write("Motor Feedback")
                # st.markdown(self.data2.to_html(index=False), unsafe_allow_html=True)
                self.table_placeholder_2 = st.empty()

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

    def haversine(lat1, lon1, lat2, lon2):
        R = 6371000  # รัศมีโลก (เมตร)
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))
        
        return R * c  # ระยะทางเป็นเมตร

    def listener_callback(self, msg):
        try:
            self.json_obj = json.loads(msg.data)
        except:
            pass

    def ping_ipv6(self, ip):
        """ใช้คำสั่ง ping6 เพื่อวัดค่า Latency ของ IPv6"""
        try:
            result = subprocess.run(["ping6", "-c", "1", ip], capture_output=True, text=True)
            for line in result.stdout.split("\n"):
                if "time=" in line:
                    latency = line.split("time=")[1].split(" ")[0]
                    return float(latency)
        except Exception as e:
            return None
        return None
        
    def timer_callback(self):
        self.target_ip = "fc94:9aac:62de:71ae:5db7:fead:ed95:e368"
        latency = self.ping_ipv6(self.target_ip)
        if latency is not None:
            if latency < 50:
                color = "🟢"
                text_color = "green"
            elif latency < 150:
                color = "🟡"
                text_color = "orange"
            else:
                color = "🔴"
                text_color = "red"

            self.table_placeholder_3.markdown(
                f"<div style='text-align: left; font-weight: bold; color: {text_color};'>{color} {latency:.2f} ms</div>",
                unsafe_allow_html=True
            )
        else:
            self.table_placeholder_3.markdown(
                "<div style='text-align: left; font-weight: bold; color: red;'>⚠️ Failed to get latency data</div>",
                unsafe_allow_html=True
            )

        try:
            self.data1 = pd.DataFrame({
                'Name': ['Value'],
                'CPU Temp': [self.json_obj.get("temps", {}).get("rct")],
                'Temp 1': [self.json_obj.get("temps", {}).get("rht1")],
                'Temp 2': [self.json_obj.get("temps", {}).get("rht2")],
            })
        except:
            self.data1 = pd.DataFrame({
                'Name': ['Value'],
                'CPU Temp': ['N/A'],
                'Temp 1': ['N/A'],
                'Temp 2': ['N/A'],
            })
            pass
        t1 = self.data1.to_html(index=False, border=0)
        t1 = t1.replace('<table ', '<table style="text-align: center;" ')
        t1 = t1.replace('<th>', '<th style="text-align: center;">')
        t1 = t1.replace('<td>', '<td style="text-align: center;">')
        self.table_placeholder_1.markdown(t1, unsafe_allow_html=True)
        
        try:
            # Create the DataFrame with Name column
            self.data2 = pd.DataFrame({
                'Name': ["wheel_fr", "wheel_fl", "wheel_rr", "wheel_rl", "str_fr", "str_fl", "str_rr", "str_rl"]
            })

            # Map motor data from JSON
            motor_data = {name: self.json_obj["motor"].get(name, None) for name in self.data2['Name']}

            # Map the motor data from 1/0 to green/red symbols
            self.data2['Status'] = self.data2['Name'].map(
                lambda name: "🟢" if motor_data.get(name) == 1 else ("🔴" if motor_data.get(name) == 0 else "N/A")
            )

        except Exception as e:
            self.data2 = pd.DataFrame({
                'Name': ["wheel_fr", "wheel_fl", "wheel_rr", "wheel_rl", "str_fr", "str_fl", "str_rr", "str_rl"],
                'Status': ['N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A'],
            })
            print(f"Error: {e}")

        # Convert the data to HTML table format
        t2 = self.data2.to_html(index=False, border=0)
        t2 = t2.replace('<table ', '<table style="text-align: center;" ')
        t2 = t2.replace('<th>', '<th style="text-align: center;">')
        t2 = t2.replace('<td>', '<td style="text-align: center;">')

        # Display the table
        self.table_placeholder_2.markdown(t2, unsafe_allow_html=True)

        try:
            self.latitude = float(self.json_obj["gps"][0])
            self.longitude = float(self.json_obj["gps"][1])
        except:
            self.latitude = 0
            self.longitude = 0
            pass

        station_latitude = 13.901335
        station_longitude = 100.532242

        now = self.get_clock().now()
        if self.last_time is None or ((now - self.last_time).nanoseconds / 1e9) >= 1.0:
            self.last_time = now
            # Set the viewport location
            try:
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

                # สร้างข้อมูลเส้นระยะ
                line_data = pd.DataFrame({
                    'from_lat': [station_latitude],
                    'from_lon': [station_longitude],
                    'to_lat': [self.latitude],
                    'to_lon': [self.longitude]
                })

                # Layer สำหรับแสดงเส้นระยะ
                line_layer = pdk.Layer(
                    'LineLayer',
                    line_data,
                    get_source_position='[from_lon, from_lat]',
                    get_target_position='[to_lon, to_lat]',
                    get_width=2,  # ความหนาของเส้น
                    get_color=[255, 0, 0],  # สีแดง
                    pickable=True
                )

                # คำนวณระยะทาง
                # distance_meters = self.haversine(station_latitude, station_longitude, self.latitude, self.longitude)
                # distance_text = f"{distance_meters:.2f} m"

                # text_data = pd.DataFrame({
                #     'lat': [(station_latitude + self.latitude) / 2],  # จุดกึ่งกลางระหว่างสถานีและตำแหน่งปัจจุบัน
                #     'lon': [(station_longitude + self.longitude) / 2],
                #     'text': [distance_text]
                # })

                # text_layer = pdk.Layer(
                #     "TextLayer",
                #     text_data,
                #     get_position='[lon, lat]',
                #     get_text='text',
                #     get_size=24,  # ขนาดตัวอักษร
                #     get_color=[255, 255, 255],  # สีขาว
                #     get_alignment_baseline='"middle"',
                # )

                # Create and render the deck with Mapbox satellite style
                r = pdk.Deck(
                    layers=[icon_layer, line_layer],  # Icon layer should be placed over the map
                    initial_view_state=view_state,
                    map_style=map_style,  # Apply the satellite style
                )

                # Display the map in Streamlit
                self.map_placeholder.pydeck_chart(r)
            except:
                pass

def main(args=None):
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
