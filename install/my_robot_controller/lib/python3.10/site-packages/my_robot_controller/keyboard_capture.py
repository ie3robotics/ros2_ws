import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select
import signal

class KeyboardController(Node):

    def __init__(self):
        super().__init__("keyboard_capture")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.keyboard_check)
        self.last_key = None
        self.stop_msg = Twist()  # Create a stop message with 0 velocities

        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        self.get_logger().info("Keyboard capture node started")

    def keyboard_check(self):
        msg = Twist()
        dr, dw, de = select.select([sys.stdin], [], [], 0.1) #checks if there is an availble key to read

        if dr:
            key = sys.stdin.read(1)
            self.last_key = key

        if self.last_key == 'w':
            msg.linear.x = 1.0
        elif self.last_key == 's':
            msg.linear.x = -1.0
        elif self.last_key == 'a':
            msg.angular.z = 1.0
        elif self.last_key == 'd':
            msg.angular.z = -1.0
        else:
            msg = self.stop_msg  # Send stop message if no valid key is pressed
            self.last_key = None  # Reset last key if no valid key is pressed

        self.cmd_vel_pub_.publish(msg)

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()

def signal_handler(sig, frame):
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    
    # Register the signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
