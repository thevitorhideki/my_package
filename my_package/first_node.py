import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np

"""
ros2 launch my_package first_node.launch.py
"""

class FirstNode(Node):

    def __init__(self):
        super().__init__('first_node')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.25, self.control)

    def control(self):
        self.twist = Twist()
        self.twist.linear.x = -0.2

        self.vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    first_node = FirstNode()

    rclpy.spin(first_node)

    first_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
