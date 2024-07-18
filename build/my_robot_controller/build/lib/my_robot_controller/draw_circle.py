#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Transbot_Lib import Transbot
from pynput import keyboard
import time


class DrawCircleNode(Node):


   def __init__(self):
       super().__init__("draw_circle")
       self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
       self.timer = self.create_timer(0.01, self.send_velocity_command)  # Adjusted timer interval
       self.bot = Transbot()  # Initialize the bot here
       self.current_key = None
       self.linear_velocity = 0.0
       self.angular_velocity = 0.0
       self.get_logger().info("Draw circle node has been started")


       self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
       self.listener.start()


   def send_velocity_command(self):
       msg = Twist()
       msg.linear.x = self.linear_velocity
       msg.angular.z = self.angular_velocity
       self.set_car_motion(self.linear_velocity * 50, self.angular_velocity * 50)




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
           if key == keyboard.KeyCode.from_char('w'):
               self.linear_velocity = 2.0
               self.angular_velocity = 2.0
              
           elif key == keyboard.KeyCode.from_char('s'):
               self.linear_velocity = -2.0
               self.angular_velocity = -2.0
              
           elif key == keyboard.KeyCode.from_char('d'):
               self.linear_velocity = 2.0
               self.angular_velocity = -2.0
              
           elif key == keyboard.KeyCode.from_char('a'):
               self.linear_velocity = -2.0
               self.angular_velocity = 2.0
              
       except AttributeError:
           pass


   def on_release(self, key):
       self.linear_velocity = 0.0
       self.angular_velocity = 0.0
      




def main(args=None):
   rclpy.init(args=args)
   node = DrawCircleNode()
   rclpy.spin(node)
   rclpy.shutdown()




if __name__ == "__main__":
   main()





