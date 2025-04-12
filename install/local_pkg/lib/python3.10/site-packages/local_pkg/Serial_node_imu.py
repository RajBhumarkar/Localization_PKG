#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String  # or a custom message type
from sensor_msgs.msg import Imu
# import tf_conversions
class SerialNodeIMU(Node):
    def __init__(self):
        super().__init__('serial_node')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.publisher_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.serial_port.in_waiting:
            
            line = self.serial_port.readline().decode('utf-8').strip()
            if line.startswith("Q:"):
                parts = line[2:].split(',')
                if len(parts) == 4:
                    qx = float(parts[0])
                    qy = float(parts[1])
                    qz = float(parts[2])
                    qw = float(parts[3])
            # line = float(line)
        
            # Process line and publish
            # print(line)
            
                    msg = Imu()
                    msg.orientation.x =  qx
                    msg.orientation.y =  qy
                    msg.orientation.z =  qz
                    msg.orientation.w =  qw

            # msg = Imu()
            # msg.orientation.z = line
            
            
            # msg. = line  # or process as needed
            # self.publisher_.publish(msg)
                    self.publisher_imu.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNodeIMU()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()