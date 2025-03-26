#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Float32MultiArray, Header
#from magnotether.magnotether import AnalogInputs  # Import the custom message
from uldaq import get_daq_device_inventory, DaqDevice, InterfaceType, AiInputMode, AInFlag
#from ros2daq.msg import AnalogInputs

class AnalogPublisher(Node):
    def __init__(self):
        super().__init__('analog_publisher')

        # Initialize DAQ device
        devices = get_daq_device_inventory(InterfaceType.USB)
        if not devices:
            self.get_logger().error("No USB DAQ devices found.")
            return

        self.daq_device = DaqDevice(devices[0])  # Connect to the first available device
        self.ai_device = self.daq_device.get_ai_device()
        self.daq_device.connect()

        # Get device range (modify based on your hardware)
        ai_info = self.ai_device.get_info()
        self.ai_range = ai_info.get_ranges(AiInputMode.SINGLE_ENDED)[0]

        self.publisher_ = self.create_publisher(Float32MultiArray, 'analog_inputs', 10)
        self.timer = self.create_timer(.01, self.publish_analog_data)  # Publish every 10ms

    def publish_analog_data(self):
        if not self.daq_device.is_connected():
            self.get_logger().error("DAQ device not connected.")
            return
        data =[0.]5
        msg = msg = Float32MultiArray()

        try:
            flags = AInFlag.DEFAULT
            # Read analog inputs from channels 0 to 7
            #t1 =time.time()
            data[0] = (time.time())
            data[1] = self.ai_device.a_in(0, AiInputMode.SINGLE_ENDED, self.ai_range, flags)
            data[2] = self.ai_device.a_in(1, AiInputMode.SINGLE_ENDED, self.ai_range, flags)
            data[3] = self.ai_device.a_in(2, AiInputMode.SINGLE_ENDED, self.ai_range, flags)
            #data[4] = self.ai_device.a_in(3, AiInputMode.SINGLE_ENDED, self.ai_range, flags)
           # data[5] = self.ai_device.a_in(4, AiInputMode.SINGLE_ENDED, self.ai_range, flags)
            #data[6] = self.ai_device.a_in(5, AiInputMode.SINGLE_ENDED, self.ai_range, flags)
            #data[7] = self.ai_device.a_in(6, AiInputMode.SINGLE_ENDED, self.ai_range, flags)
           # data[8] = self.ai_device.a_in(7, AiInputMode.SINGLE_ENDED, self.ai_range, flags)
            #t2 = time.time()
            print(t2-t1)
            #msg.header.stamp = self.get_clock().now().to_msg()
            msg.data=data
            self.publisher_.publish(msg)
            #self.get_logger().info(f'Published: {data}')


        except Exception as e:
            self.get_logger().error(f"Error reading from DAQ device: {e}")

    def destroy_node(self):
        # Disconnect the DAQ device before shutting down
        if self.daq_device.is_connected():
            self.daq_device.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AnalogPublisher()
    if node.daq_device.is_connected():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()