=#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from uldaq import (
    get_daq_device_inventory, DaqDevice, InterfaceType,
    AiInputMode, AInFlag, AOutFlag, Range
)


class DAQNode(Node):
    def __init__(self):
        super().__init__('daq_node')

        # === Init DAQ Device ===
        devices = get_daq_device_inventory(InterfaceType.USB)
        if not devices:
            self.get_logger().error("No USB DAQ devices found.")
            return

        self.daq_device = DaqDevice(devices[0])
        self.daq_device.connect()

        # === Analog Input Setup ===
        self.ai_device = self.daq_device.get_ai_device()
        ai_info = self.ai_device.get_info()
        self.ai_range = list(ai_info.get_ranges(AiInputMode.SINGLE_ENDED))[0]
        self.num_ai_channels = 8

        self.ai_publishers = [
            self.create_publisher(Float32, f'/analog_input/AI{ch}', 10)
            for ch in range(self.num_ai_channels)
        ]

        self.create_timer(0.01, self.read_and_publish_ai)  # 100Hz

        # === Analog Output Setup ===
        self.ao_device = self.daq_device.get_ao_device()
        if self.ao_device:
            self.ao_range = Range.UNI5VOLTS  # Adjust to match your hardware
            self.min_voltage = 0.0
            self.max_voltage = 5.0

            self.create_subscription(
                Float32MultiArray,
                '/analog_output/command',
                self.handle_ao_command,
                10
            )
            self.get_logger().info("Analog output enabled.")
        else:
            self.get_logger().warn("No AO device found.")

    def read_and_publish_ai(self):
        flags = AInFlag.DEFAULT

        for ch in range(self.num_ai_channels):
            try:
                voltage = self.ai_device.a_in(ch, AiInputMode.SINGLE_ENDED, self.ai_range, flags)
                self.ai_publishers[ch].publish(Float32(data=voltage))
            except Exception as e:
                self.get_logger().error(f"Error reading AI{ch}: {e}")

    def handle_ao_command(self, msg: Float32MultiArray):
        if not self.ao_device:
            self.get_logger().error("AO device not available.")
            return

        if len(msg.data) < 2:
            self.get_logger().warn("AO command needs [channel, voltage].")
            return

        ch = int(msg.data[0])
        voltage = float(msg.data[1])
        voltage = max(self.min_voltage, min(voltage, self.max_voltage))  # Clamp

        try:
            self.ao_device.a_out(ch, self.ao_range, AOutFlag.DEFAULT, voltage)
            self.get_logger().info(f"Set AO{ch} to {voltage:.2f} V")
        except Exception as e:
            self.get_logger().error(f"Failed to write AO{ch}: {e}")

    def destroy_node(self):
        if self.daq_device.is_connected():
            self.daq_device.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DAQNode()
    if node.daq_device.is_connected():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
