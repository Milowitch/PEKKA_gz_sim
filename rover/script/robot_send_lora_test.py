import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class MotorPublisher(Node):
    def __init__(self):
        super().__init__('motor_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        self.publisher_ = self.create_publisher(String, '/motor_msg/rawdata', qos_profile)  # Topic name: /motor_msg/rawdata
        timer_period = 1  # 1 second
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Motor publisher node is running.')

    def generate_random_motor_data(self):
        motor_data = []
        motor_names = ['mg_1', 'mg_2', 'mg_3', 'mg_4', 'sm_5', 'sm_6', 'sm_7', 'sm_8']
        
        for name in motor_names:
            motor = {
                "name": name,
                "effort": round(random.uniform(-1, 1), 6),  # Random effort between -1 and 1
                "position": round(random.uniform(0, 360), 4),  # Random position between 0 and 360 degrees
                "tempareture": random.randint(25, 40),  # Random temperature between 25 and 40 degrees Celsius
                "velocity": round(random.uniform(0, 100), 2),  # Random velocity between 0 and 100
                "voltage": random.randint(110, 120),  # Random voltage between 110 and 120V (for some motors)
                "current": random.randint(1, 30),  # Random current between 1 and 30A
                "move": random.choice([0, 1])  # Random move status (0 = not moving, 1 = moving)
            }
            motor_data.append(motor)

        return motor_data

    def timer_callback(self):
        motor_data = self.generate_random_motor_data()

        # Convert the motor data list to a JSON string
        motor_data_json = json.dumps(motor_data)

        # Create a String message and publish it
        msg = String()
        msg.data = motor_data_json
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published motor data: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    motor_publisher = MotorPublisher()
    rclpy.spin(motor_publisher)
    motor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
