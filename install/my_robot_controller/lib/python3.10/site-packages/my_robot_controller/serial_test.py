#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Transbot_Lib import Transbot
import time
import serial

class SerialConnect(Node):
    def __init__(self):
        super().__init__("serial_test")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/distance", 10)
        self.timer = self.create_timer(0.1, self.serial_connect)  # Adjusted timer interval to 0.1 seconds
        self.get_logger().info("Distance test started1")

        # Initialize the serial connection here
        self.arduino_ports = ['/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM2']
        self.baud_rate = 115200
        self.ser = None

        for port in self.arduino_ports:
            try:
                self.ser = serial.Serial(port, self.baud_rate, timeout=1)
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
                self.ser.flushInput()  # Flush the input buffer to remove old data
                line = self.ser.readline().decode().strip()

                self.get_logger().info(f"Received: {line}")

                
                msg = Twist()
                msg.linear.y = float(line)
                self.cmd_vel_pub_.publish(msg)

            except ValueError as e:
                self.get_logger().error(f"Error converting distance value: {str(e)}")
            except serial.SerialException as e:
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
