#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
import cv_bridge
import cv2
import numpy as np
from datetime import datetime
import os
import threading
import time

class Payload(Node):
    def __init__(self):
        super().__init__('payload')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        self.bridge = cv_bridge.CvBridge()
        
        # Subscriptions for image topics
        self.subscription_0 = self.create_subscription(
            CompressedImage,
            'image_raw/compressed0',
            self.image_callback_0,
            qos_profile
        )
        self.subscription_1 = self.create_subscription(
            CompressedImage,
            'image_raw/compressed1',
            self.image_callback_1,
            qos_profile
        )
        
        # Subscription for the control topic
        self.control_subscription = self.create_subscription(
            Bool,
            '/record_control',
            self.control_callback,
            qos_profile
        )
        
        # Video writers
        self.video_writer_0 = None
        self.video_writer_1 = None
        
        # Record state
        self.recording = False
        self.recording_started = False  # Flag to track if recording has started

        self.comp_image_0 = CompressedImage()
        self.comp_image_1 = CompressedImage()

        self.process_thread = threading.Thread(target=self.process_thread)
        self.process_thread.start()

    def image_callback_0(self, msg):
        self.comp_image_0 = msg

    def image_callback_1(self, msg):
        self.comp_image_1 = msg

    def control_callback(self, msg):
        self.recording = msg.data
        if not self.recording:
            # If recording is turned off, release video writers
            self.get_logger().info('Recording stopped')
            self.release_video_writers()
            self.recording_started = False
        else:
            # If recording is turned on, initialize video writers if not already started
            if not self.recording_started:
                self.get_logger().info('Recording started')
                self.recording_started = True

    def process_image(self, msg, camera_id):
        if not self.recording:
            return
        
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if camera_id == 0:
                if self.video_writer_0 is None:
                    self.init_video_writer(cv_image, 0)
                self.video_writer_0.write(cv_image)
            elif camera_id == 1:
                if self.video_writer_1 is None:
                    self.init_video_writer(cv_image, 1)
                self.video_writer_1.write(cv_image)
        except Exception as e:
            self.get_logger().error('Error processing image from camera %d: %s' % (camera_id, str(e)))

    def init_video_writer(self, image, camera_id):
        try:
            height, width, _ = image.shape
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            video_format = 'mp4'
            directory = os.path.expanduser('~/files/')
            os.makedirs(directory, exist_ok=True)
            video_filename = f'{directory}output_video_{camera_id}_{timestamp}.{video_format}'
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = 30
            if camera_id == 0:
                self.video_writer_0 = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))
            elif camera_id == 1:
                self.video_writer_1 = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))
        except Exception as e:
            self.get_logger().error('Error initializing video writer for camera %d: %s' % (camera_id, str(e)))

    def release_video_writers(self):
        if self.video_writer_0 is not None:
            self.video_writer_0.release()
            self.video_writer_0 = None
        if self.video_writer_1 is not None:
            self.video_writer_1.release()
            self.video_writer_1 = None
    
    def process_thread(self):
        while rclpy.ok():
            try:
                self.process_image(self.comp_image_0, 0)
                self.process_image(self.comp_image_1, 1)
            except:
                self.get_logger().error(f'Can not read video')
                continue
            time.sleep(1/30)

    def destroy_node(self):
        self.release_video_writers()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    payload = Payload()
    try:
        rclpy.spin(payload)
    except KeyboardInterrupt:
        pass
    
    payload.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
