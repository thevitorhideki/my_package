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


class Testa(Node, Odom, Laser):  # Mude o nome da classe

    def __init__(self):
        Node.__init__(self, 'testa_tudo')  # Mude o nome do nó
        Odom.__init__(self)
        Laser.__init__(self)

        self.bridge = CvBridge()
        self.kernel = np.ones((5, 5), np.uint8)

        self.yellow = {"lower": (20, 40, 40), "upper": (30, 255, 255)}

        self.robot_state = 'gira'
        self.state_machine = {
            "stop": self.stop,
            "segue_linha": self.segue_linha,
            "vai_para": self.vai_para,
            "gira": self.gira
        }

        # Inicialização de variáveis
        self.inicio_x = 0
        self.inicio_y = 0
        self.voltou = False
        self.centro_imagem = 0
        self.centro_x_amarelo = 0
        self.kp_angular = 0.005
        self.twist = Twist()
        self.goto = GoTo(Point())
        self.rotaciona = RotateTo(180, deg=True)

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

        time.sleep(1)
        self.timer = self.create_timer(0.25, self.control)

    def image_callback(self, msg: np.ndarray):
        cv_image = self.bridge.imgmsg_to_cv2(
            msg, "bgr8"
        )

        h, w, _ = cv_image.shape
        self.centro_imagem = w / 2

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.yellow["lower"], self.yellow["upper"])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        mask[: int(h / 2), :] = 0

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Encontra o maior contorno e calcula o centro
            contour = max(contours, key=cv2.contourArea)

            M = cv2.moments(contour)
            self.centro_x_amarelo = int(M["m10"] / M["m00"])
            self.centro_amarelo_y = int(M["m01"] / M["m00"])

        elif len(contours) == 0:
            self.centro_x_amarelo = np.inf
            self.centro_amarelo_y = np.inf

        cv2.imshow("Linha", mask)
        cv2.waitKey(1)

    def stop(self):
        self.twist = Twist()

    def distancia(self):
        return np.sqrt((self.inicio_x - self.x)**2 + (self.inicio_y - self.y)**2)

    def segue_linha(self):
        if not self.inicio_x:
            self.inicio_x = self.x
            self.inicio_y = self.y

        erro = self.centro_imagem - self.centro_x_amarelo

        self.twist.linear.x = 0.5
        self.twist.angular.z = erro * self.kp_angular

        if self.distancia() <= 0.5 and self.voltou:
            self.robot_state = "stop"
        elif self.distancia() >= 0.5:
            self.voltou = True

        if self.centro_x_amarelo == np.inf and self.x:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.2

    def vai_para(self):
        self.goto.point = Point(x=1.0, y=0.0)
        if not self.goto.robot_state == 'stop':
            rclpy.spin_once(self.goto)
            self.twist = self.goto.twist
        else:
            self.robot_state = 'segue_linha'

    def gira(self):
        if not self.rotaciona.robot_state == 'para':
            rclpy.spin_once(self.rotaciona)
            self.twist = self.rotaciona.twist
        else:
            self.robot_state = 'vai_para'

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = Testa()  # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
