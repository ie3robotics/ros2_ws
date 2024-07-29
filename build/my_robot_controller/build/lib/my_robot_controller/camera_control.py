#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
import numpy as np
from pynput import keyboard
from Transbot_Lib import Transbot

class CamController(Node):

    def __init__(self):
        super().__init__('camera_control')

    
        # Parameters and initial setup
        self.bot = Transbot() 
        self.pwm_servo_x = 90
        self.pwm_servo_y = 25
        self.init_servos()

        # Start video capture
        self.capture = cv2.VideoCapture(0)

        # Create a timer to periodically call the `show_camera_feed` method
        self.timer = self.create_timer(0.00001, self.show_camera_feed)

        # Start keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()


    def init_servos(self):
        self.bot.set_pwm_servo(1, self.pwm_servo_x)
        self.bot.set_pwm_servo(2, self.pwm_servo_y)
        self.get_logger().info(f'Servos initialized: Servo1={self.pwm_servo_x}, Servo2={self.pwm_servo_y}')

    def on_press(self, key):
        try:
            if key == keyboard.Key.up:
                self.pwm_servo_y = np.clip(self.pwm_servo_y + 5, 0, 180)
            elif key == keyboard.Key.down:
                self.pwm_servo_y = np.clip(self.pwm_servo_y - 5, 0, 180)
            elif key == keyboard.Key.right:
                self.pwm_servo_x = np.clip(self.pwm_servo_x - 5, 0, 180)
            elif key == keyboard.Key.left:
                self.pwm_servo_x = np.clip(self.pwm_servo_x + 5, 0, 180)

            self.bot.set_pwm_servo(1, int(self.pwm_servo_x))
            self.bot.set_pwm_servo(2, int(self.pwm_servo_y))
            self.get_logger().info(f'Servos moved: Servo1={int(self.pwm_servo_x)}, Servo2={int(self.pwm_servo_y)}')
        except AttributeError:
            pass

    def show_camera_feed(self):
        ret, frame = self.capture.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return

        cv2.imshow('Camera Feed', frame)
        cv2.waitKey(1)




def main(args=None):
    rclpy.init(args=args)
    node = CamController()
    rclpy.spin(node)
    node.capture.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
