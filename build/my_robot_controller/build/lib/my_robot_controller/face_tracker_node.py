#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
import numpy as np
from Transbot_Lib import Transbot
from pynput import keyboard

class CameraControlNode(Node):

    def __init__(self):
        super().__init__('face_tracker_node')

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_object_detected = self.create_publisher(String, '/object_detected', 10)

        # Parameters and initial setup
        self.bot = Transbot()  # Initialize the bot here
        self.pwm_servo_x = 90
        self.pwm_servo_y = 25
        self.init_servos()

        # Start video capture
        self.capture = cv2.VideoCapture(0)

        # Object tracking
        self.tracker = cv2.TrackerKCF_create() if hasattr(cv2, 'TrackerKCF_create') else None
        if not self.tracker:
            self.get_logger().error('TrackerKCF_create not found in cv2. Please ensure you have opencv-contrib-python installed.')
            raise AttributeError('TrackerKCF_create not found in cv2')
        self.bbox = None
        self.tracking = False

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

        if not self.tracking:
            # Detect an object (using any method, here we use manual selection for simplicity)
            self.bbox = cv2.selectROI('Camera Feed', frame, fromCenter=False, showCrosshair=True)
            self.tracker.init(frame, self.bbox)
            self.tracking = True

        else:
            # Update the tracker
            success, self.bbox = self.tracker.update(frame)
            if success:
                p1 = (int(self.bbox[0]), int(self.bbox[1]))
                p2 = (int(self.bbox[0] + self.bbox[2]), int(self.bbox[1] + self.bbox[3]))
                cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)

                # Calculate distance (this is a placeholder, replace with actual calculation)
                distance = self.calculate_distance(self.bbox)

                if 6 <= distance <= 12:
                    self.publish_object_detected()

        cv2.imshow('Camera Feed', frame)
        cv2.waitKey(1)

    def calculate_distance(self, bbox):
        # Placeholder function for distance calculation
        # Replace with actual distance measurement logic
        # Assuming the object is a known size and using bounding box size to estimate distance
        known_width = 5.0  # inches (for example)
        focal_length = 800  # placeholder value
        width_in_frame = bbox[2]
        distance = (known_width * focal_length) / width_in_frame
        return distance

    def publish_object_detected(self):
        msg = String()
        msg.data = "Object detected within 6-12 inches"
        self.pub_object_detected.publish(msg)
        self.get_logger().info('Object detected within 6-12 inches')

def main(args=None):
    rclpy.init(args=args)
    node = CameraControlNode()
    rclpy.spin(node)
    node.capture.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
