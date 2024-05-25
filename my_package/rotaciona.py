import time
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from my_package.odom import Odom

# Adicione aqui os imports necessários
import numpy as np


class RotateTo(Node, Odom):

    def __init__(self, target):
        Node.__init__(self, 'rotate_node')
        Odom.__init__(self)

        time.sleep(1)

        self.timer = self.create_timer(0.2, self.control)

        self.robot_state = 'gira'
        self.state_machine = {
            'gira': self.gira,
            'para': self.para
        }

        # Inicialização de variáveis
        self.kp = 0.8
        self.twist = Twist()
        self.get_goal_from_target(target)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def get_goal_from_target(self, target):
        self.goal_yaw = (target + np.pi) % (2 * np.pi) - np.pi

    def gira(self):
        erro = self.goal_yaw - self.yaw
        erro = np.arctan2(np.sin(erro), np.cos(erro))

        self.twist.angular.z = erro * self.kp

        if abs(erro) <= np.deg2rad(10):
            self.robot_state = 'para'

    def para(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = RotateTo(np.deg2rad(0))

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
