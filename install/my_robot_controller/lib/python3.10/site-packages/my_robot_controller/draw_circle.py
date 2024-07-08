#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Transbot_Lib import Transbot
from pynput import keyboard


class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.05, self.send_velocity_command)
        self.bot = Transbot()  # Initialize the bot here
        self.current_key = None
        self.get_logger().info("Draw circle node has been started")


        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()


    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0


        if self.current_key == 'w':
            self.set_car_motion(50,50)
        elif self.current_key == 's':
            self.set_car_motion(-50,-50)
        elif self.current_key == 'a':
            self.set_car_motion(-50,50)
        elif self.current_key == 'd':
            self.set_car_motion(50,-50)
        else:
            self.set_car_motion(0,0)



        
        self.cmd_vel_pub_.publish(msg)

    def set_car_motion(self, A, B):
        if abs(A) >= 20 or abs(B) >= 20: 
            self.bot.set_motor(1, A)
            self.bot.set_motor(2, B)
            return A, B
        else:
            self.bot.set_motor(1, 0)
            self.bot.set_motor(2, 0)
            return 0, 0

    def on_press(self, key):
        try:
            self.current_key = key.char
        except AttributeError:
            pass

    def on_release(self, key):
        self.current_key = None

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
