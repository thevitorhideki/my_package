import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from my_package.odom import Odom
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import ReliabilityPolicy, QoSProfile


class Seguidor(Node, Odom):
    def __init__(self):
        Node.__init__(self, "seguidor_node")
        Odom.__init__(self)

        # Configura a camera
        self.bridge = CvBridge()
        self.kernel = np.ones((5, 5), np.uint8)

        # Máscaras
        self.yellow = {"lower": (20, 40, 40), "upper": (30, 255, 255)}

        self.timer = self.create_timer(0.1, self.control)

        self.robot_state = "segue_linha"
        self.state_machine = {
            "stop": self.stop,
            "segue_linha": self.segue_linha,
        }

        # Inicialização de variáveis
        self.inicio_x = 0
        self.inicio_y = 0
        self.voltou = False

        # Variáveis de controle
        self.twist = Twist()
        self.kp_angular = 0.005

        # Subscribers
        # Coloque aqui os subscribers
        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

        # Publishers
        self.cmd_vel_pub2 = self.create_publisher(Twist, "cmd_vel", 10)
        # Coloque aqui os publishers

    def image_callback(self, msg):
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

        self.twist.linear.x = 1.0
        self.twist.angular.z = erro * self.kp_angular

        if self.distancia() <= 0.5 and self.voltou:
            self.robot_state = "stop"
        elif self.distancia() >= 0.5:
            self.voltou = True

        if self.centro_x_amarelo == np.inf and self.x:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.2

    def control(self):
        self.twist = Twist()
        print(f"Estado Atual: {self.robot_state}")
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub2.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = Seguidor()  # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
