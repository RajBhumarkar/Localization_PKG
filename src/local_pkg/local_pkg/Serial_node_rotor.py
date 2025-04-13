#!/ usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String 
from nav_msgs.msg import Odometry  # or a custom message type
from geometry_msgs.msg import Twist, Pose, Quaternion, Point
import numpy as np
import re
class SerialNodeRotor(Node):
    def __init__(self):
        super().__init__('serial_node_rotor')
        self.publisher_odo = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.publisher_ = self.create_publisher(String,'topic',10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
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
            line = str(line)
            self.publisher_.publish(msg)

            linear_vel = self.extract_linear_velocity(line)
            angular_vel = self.extract_angular_velocity(line)

            if linear_vel is not None or angular_vel is not None:
                output = f"Linear: {linear_vel:.2f} m/s | Angular: {angular_vel:.2f} rad/s"
                msg = Odometry()
                msg.twist.twist.linear.x = linear_vel
                msg.twist.twist.angular.z = angular_vel

                self.publisher_odo.publish(msg)
            
                self.get_logger().info(f"Published: {output}")

    def extract_linear_velocity(self, text):
            match = re.search(r'Linear Velocity:\s*([-+]?[0-9]*\.?[0-9]+)', text)
            if match:
                return float(match.group(1))
            return 0.0

    def extract_angular_velocity(self, text):
            match = re.search(r'Angular Velocity:\s*([-+]?[0-9]*\.?[0-9]+)', text)
            if match:
                deg_per_sec = float(match.group(1))
                return deg_per_sec * 3.14159 / 180.0  # degrees/s to radians/s
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNodeRotor()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()