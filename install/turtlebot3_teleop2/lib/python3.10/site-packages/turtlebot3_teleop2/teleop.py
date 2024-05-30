import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, os
from std_srvs.srv import Empty
import select

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios


LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1

msg = """
Controlando o turtlebot
---------------------------
Para movimentação:
        w
   a    s    d
        

w/s : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key: force stop

CTRL-C to quit
"""

class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.settings = termios.tcgetattr(sys.stdin) if os.name != 'nt' else None
        self.srv = self.create_service(Empty, 'stop_robot', self.stop_robot_callback)
        print(msg)

    def get_key(self):
        if os.name == 'nt':
            if msvcrt.kbhit():
                return msvcrt.getch().decode()
            return ''
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def timer_callback(self):
        key = self.get_key()
        if key == 'w':
            self.lin_vel = +LIN_VEL_STEP_SIZE
        elif key == 's':
            self.lin_vel = -LIN_VEL_STEP_SIZE
        elif key == 'a':
            self.ang_vel = ANG_VEL_STEP_SIZE
        elif key == 'd':
            self.ang_vel = -ANG_VEL_STEP_SIZE
        elif key == ' ':
            self.call_stop_service()
        elif key == '\x03':
            self.destroy_node()
            rclpy.shutdown()
            return
        
        twist = Twist()
        twist.linear.x = self.lin_vel
        twist.angular.z = self.ang_vel
        self.publisher_.publish(twist)
        print(f'Linear Velocity: {self.lin_vel}, Angular Velocity: {self.ang_vel}')

    def call_stop_service(self):
        client = self.create_client(Empty, 'stop_robot')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        req = Empty.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Robot stopped successfully by service call.')
        else:
            self.get_logger().error('Failed to call service stop_robot.')

    def stop_robot_callback(self, request, response):
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        twist = Twist()
        twist.linear.x = self.lin_vel
        twist.angular.z = self.ang_vel
        self.publisher_.publish(twist)
        print("Robot stopped by service call.")
        self.destroy_node()
        rclpy.shutdown()
        return response

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
