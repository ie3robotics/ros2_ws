import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Transbot_Lib import Transbot

class MotorController(Node):

    def __init__(self):
        super().__init__("motor_controller")
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        
        self.bot = Transbot()  # Initialize the bot here
        self.speedl = 0
        self.speedr = 0
        self.get_logger().info("Motor Controller node started")

    def listener_callback(self, msg):
        self.get_logger().info("Received Twist message with linear x: " + str(msg.linear.x))

        if msg.linear.x != 0.0 and msg.angular.z == 0.0:  # Forward or backward
            speed = msg.linear.x * 100  # Adjust the scaling factor as necessary
            self.speedl = speed
            self.speedr = speed
        elif msg.linear.x == 0.0 and msg.angular.z != 0.0:  # Left or right turn
            speed = msg.angular.z * 50  # Adjust the scaling factor as necessary
            self.speedl = -speed
            self.speedr = speed
        else:  # Stop
            self.speedl = 0
            self.speedr = 0


        self.set_car_motion(self.speedl, self.speedr)

    def set_car_motion(self, A, B):
        self.get_logger().info(f"Setting motor A(left) to {A} and motor B(right) to {B}")
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
