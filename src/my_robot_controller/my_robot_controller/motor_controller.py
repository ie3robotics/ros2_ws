import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from Transbot_Lib import Transbot
import serial
import time

class MotorController(Node):

    def __init__(self):
        super().__init__("motor_controller")
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription_distance = self.create_subscription(
            Twist,
            '/distance',
            self.distance_callback,
            10
        )
        self.subscription_joy = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        self.bot = Transbot()  # Initialize the bot here
        self.speedl = 0
        self.speedr = 0
        self.distance = None  # Initialize distance attribute
        

        self.get_logger().info("Motor Controller node started")

    def joy_callback(self, msg):
        fowaxis = msg.axes[1]
        turnaxis = msg.axes[0]

        if self.distance < 35.0 and msg.axes[1] > 0.0:
                self.set_car_motion(-100, -100)
                time.sleep(0.5)
                self.set_car_motion(-100,100)
                time.sleep(0.5)

        self.get_logger().info("fowaxis = " + str(fowaxis))
        self.get_logger().info("turnaxis = " + str(turnaxis))

        left_motor_speed = (fowaxis * 100) - (turnaxis * 100)
        right_motor_speed = (fowaxis * 100) + (turnaxis * 100)

        left_motor_speed = max(min(left_motor_speed, 100), -100)
        right_motor_speed = max(min(right_motor_speed, 100), -100)

        self.set_car_motion(left_motor_speed, right_motor_speed)

        

    def cmd_vel_callback(self, msg):
        self.get_logger().info("Received Twist message with linear x: " + str(msg.linear.x))
        self.get_logger().info("Angular.z: " + str(msg.angular.z))
        
        if self.distance is not None:
            self.get_logger().info("Distance: " + str(self.distance))

            if self.distance < 35.0 and msg.linear.x > 0.0:
                self.speedl = 0
                self.speedr = 0
                self.set_car_motion(-100, -100)
                time.sleep(0.5)
                self.set_car_motion(-100,100)
                time.sleep(0.5)

            elif msg.linear.x != 0.0 and msg.angular.z == 0.0:  # Forward or backward
                speed = msg.linear.x * 100  # Adjust the scaling factor as necessary
                self.speedl = speed - 1
                self.speedr = speed
            elif msg.linear.x == 0.0 and msg.angular.z != 0.0:  # Left or right turn
                speed = msg.angular.z * 100  # Adjust the scaling factor as necessary
                self.speedl = -speed
                self.speedr = speed
            else:  # Stop
                self.speedl = 0
                self.speedr = 0

            self.set_car_motion(self.speedl, self.speedr)

    def distance_callback(self, msg):
        if msg.linear.y != 0:

            self.distance = msg.linear.y

    def set_car_motion(self, A, B):
        self.get_logger().info(f"Setting motor A (left) to {A} and motor B (right) to {B}")
        if abs(A) >= 20 or abs(B) >= 20:
            self.bot.set_motor(1, A)
            self.bot.set_motor(2, B)
        else:
            self.bot.set_motor(1, 0)
            self.bot.set_motor(2, 0)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
