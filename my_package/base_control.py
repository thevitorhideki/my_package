import cv2
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
from cv_bridge import CvBridge

from my_package.segue_linha import Seguidor
from my_package.rotaciona import RotateTo
from my_package.odom import Odom
from my_package.laser import Laser
from my_package.goto import GoTo


class Base(Node):  # Mude o nome da classe

    def __init__(self):
        Node.__init__(self, 'base_node')  # Mude o nome do nó

        self.timer = self.create_timer(0.1, self.control)
        self.bridge = CvBridge()

        self.robot_state = 'stop'
        self.state_machine = {
            'stop': self.stop
        }

        # Inicialização de variáveis
        self.twist = Twist()

        # Subscribers
        # Coloque aqui os subscribers
        self.image_sub = self.create_subscription(
            Image,  # or CompressedImage
            'camera/image_raw',  # or '/camera/image_raw/compressed'
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Coloque aqui os publishers

    def image_callback(self, msg: np.ndarray):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        cv2.imshow("Image", cv_image)
        cv2.waitKey(1)

    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = Base()  # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
