import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

class SineWavePublisher(Node):
    def __init__(self):
        super().__init__('sine_wave_publisher')

        self.publisher_ = self.create_publisher(Float32MultiArray, '/analog_output/command', 10)
        self.timer_period = 0.01  # 100 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.start_time = time.time()
        self.channel = 0  # Analog output channel number

        self.get_logger().info('SineWavePublisher started, publishing to /analog_output/command')

    def timer_callback(self):
        current_time = time.time() - self.start_time
        frequency = 1.0  # 1 Hz
        voltage = 2.5* (1 + np.sin(2 * np.pi * frequency * current_time))  # Range 0 to 2.5V

        msg = Float32MultiArray()
        msg.data = [float(self.channel), float(voltage)]
        msg2 = Float32MultiArray()
        msg2.data = [float(1), foat(2.5)]
        self.publisher_.publish(msg)
        self.publisher_.publish(msg2)

def main(args=None):
    rclpy.init(args=args)
    node = SineWavePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()