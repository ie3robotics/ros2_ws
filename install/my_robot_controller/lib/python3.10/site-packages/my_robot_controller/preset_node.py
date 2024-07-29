import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select
import signal
import time

class PresetController(Node):

    def __init__(self):
        super().__init__("preset_node")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/distance", 10)
        
        self.get_logger().info("Preset node started")
        self.preset()

    def preset(self):
        

        self.forward(5)
        self.right(1)
        self.forward(2)
        self.left(1)
        self.backward(5)


    def forward(self, amt):
        msg = Twist()
        
        msg.linear.x = 1.0
        self.cmd_vel_pub_.publish(msg)
        time.sleep(amt)
        msg.linear.x = 0.0
        self.cmd_vel_pub_.publish(msg)

    def backward(self, amt):
        msg = Twist()
        
        msg.linear.x = -1.0
        self.cmd_vel_pub_.publish(msg)
        time.sleep(amt)
        msg.linear.x = 0.0
        self.cmd_vel_pub_.publish(msg)

    def right(self, amt):
        msg = Twist()
        
        msg.angular.z = -1.0
        self.cmd_vel_pub_.publish(msg)
        time.sleep(amt)
        msg.angular.z = 0.0
        self.cmd_vel_pub_.publish(msg)

    def left(self, amt):
        msg = Twist()
        
        msg.angular.z = 1.0
        self.cmd_vel_pub_.publish(msg)
        time.sleep(amt)
        msg.angular.z = 0.0
        self.cmd_vel_pub_.publish(msg)





def main(args=None):
    rclpy.init(args=args)
    node = PresetController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
