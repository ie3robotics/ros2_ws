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
        self.arduino_ports = ['/dev/ttyACM0']
        self.baud_rate = 115200
        self.ser = None

        for port in self.arduino_ports:
            try:
                self.ser = serial.Serial(port, self.baud_rate)
                time.sleep(2)  # Wait for the connection to initialize
                self.get_logger().info(f"Connected to {port}")
                break
            except serial.SerialException as e:
                self.get_logger().warning(f"Failed to connect to {port}: {str(e)}")

        if not self.ser:
            self.get_logger().error("Failed to connect to any serial port")
            raise RuntimeError("Serial connection failed")

    def serial_connect(self):
        if self.ser:
            try:
                line = self.ser.readline()
                self.get_logger().info(f"Received: {line.decode().strip()}")

                # Send a message to the Arduino
                self.ser.write(b'Hello from Jetson Nano\n')
                self.get_logger().info("Sent message to Arduino")
            except Exception as e:
                self.get_logger().error(f"Error in serial connection: {str(e)}")
        else:
            self.get_logger().error("Serial port not initialized")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SerialConnect()
        rclpy.spin(node)
    except RuntimeError as e:
        rclpy.logging.get_logger("serial_test").error(f"Node initialization failed: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
