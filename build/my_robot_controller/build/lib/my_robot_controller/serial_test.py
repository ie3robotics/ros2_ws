#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Transbot_Lib import Transbot
from pynput import keyboard
import time
import serial

class SerialConnect(Node):
    def __init__(self):
        super().__init__("serial_test")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/distance", 10)
        self.timer = self.create_timer(0.05, self.serial_connect)  # Adjusted timer interval
        self.get_logger().info("Distance test started")
        
        # Initialize the serial connection here
        self.arduino_port = '/dev/ttyS1'  # Replace with the correct port
        self.baud_rate = 115200
        self.ser = serial.Serial(self.arduino_port, self.baud_rate)
        time.sleep(2)  # Wait for the connection to initialize

    def serial_connect(self):
        try:
            
            line = self.ser.readline()
            self.get_logger().info(str(line))

                
             

            # Send a message to the Arduino
            self.ser.write(b'Hello from Jetson Nano\n')
            self.get_logger().info("sent msg to aurdino")
        except Exception as e:
            self.get_logger().error(f"Error in serial connection: {str(e)}")



def main(args=None):
    rclpy.init(args=args)
    node = SerialConnect()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
