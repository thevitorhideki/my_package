import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# Adicione aqui os imports necessários
from my_package.odom import Odom
import numpy as np


class GoTo(Node, Odom):

    def __init__(self, point):
        Node.__init__(self, 'goto_node')
        Odom.__init__(self)

        time.sleep(1)
        self.timer = self.create_timer(0.2, self.control)

        self.robot_state = 'center'
        self.state_machine = {
            'center': self.center,
            'goto': self.goto,
            'stop': self.stop
        }

        self.kp_angular = 0.2
        self.kp_linear = 0.1

        # Inicialização de variáveis
        self.twist = Twist()
        self.point = point

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def get_angular_error(self):
        x, y = self.point
        dx = x - self.x
        dy = y - self.y
        theta = np.arctan2(dy, dx)
        goal_yaw = (theta + np.pi) % (2 * np.pi) - np.pi
        erro = goal_yaw - self.yaw
        erro = np.arctan2(np.sin(erro), np.cos(erro))
        dist = np.sqrt(dx**2 + dy**2)

        return erro, dist

    def center(self):
        erro, _ = self.get_angular_error()
        self.twist.angular.z = self.kp_angular * erro

        if abs(erro) < np.deg2rad(2):
            self.robot_state = 'goto'

    def goto(self):
        erro, dist = self.get_angular_error()
        self.twist.angular.z = self.kp_angular * erro
        self.twist.linear.x = self.kp_linear * dist

        if dist < 0.1:
            self.robot_state = 'stop'

    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        print(f'X: {self.x}| Y:{self.y}')
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = GoTo((0, 0))

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
