#!/ usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String  # or a custom message type
import numpy as np
class SerialNodeRotor(Node):
    def __init__(self):
        super().__init__('serial_node_rotor')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.serial_port = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.serial_port.in_waiting:
            # line = self.serial_port.readline().decode('utf-8').rstrip()
            line = self.serial_port.readline()
            # print(f"Raw data: {line}")
            # print(type(line))
            # Process line and publish
            msg = String()
            # msg.data = line  # or process as needed
            msg.data = line.decode('utf-8', errors='ignore').strip()

            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNodeRotor()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()