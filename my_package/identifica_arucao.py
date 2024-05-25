from my_package.module_aruco import Aruco3d
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2
import numpy as np

# Adicione aqui os imports necessários


class DetectaAruco(Node, Aruco3d):
    def __init__(self) -> None:
        Node.__init__(self, "arucos_node")
        Aruco3d.__init__(self)

        # Configura a camera
        self.bridge = CvBridge()
        self.kernel = np.ones((5, 5), np.uint8)

        # Inicialização de variáveis
        self.string = String()
        self.string.data = ""
        self.running = True

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,  # or CompressedImage
            "/camera/image_raw",  # or '/camera/image_raw/compressed'
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        # Coloque aqui os subscribers

        # Publishers
        self.aruco_pub = self.create_publisher(String, "/arucos", 10)
        # Coloque aqui os publishers

    def flag_callback(self, msg) -> None:
        self.running = bool(msg.data)

    def image_callback(self, msg) -> None:
        if self.running:
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, "bgr8"
            )
            # cv_image = self.bridge.compressed_imgmsg_to_cv2(
            #     msg, "bgr8"
            # )  # if CompressedImage

            bgr, aruco = self.run(cv_image)

            if aruco and aruco[0]['id'][0] >= 100:
                self.string.data = str(
                    aruco[0]['id'][0]) + ' ' + str(aruco[0]['distancia'])
                self.aruco_pub.publish(self.string)
            else:
                self.string.data = str(
                    1) + ' ' + str(1)
                self.aruco_pub.publish(self.string)
            cv2.imshow("Arucos grandes", bgr)
            cv2.waitKey(1)

        else:
            print("Image processing is paused")

    def run(self, bgr) -> list:
        bgr, results = self.detectaAruco(bgr)

        if results:
            return [bgr, results]

        return [bgr, None]


def main(args=None) -> None:
    rclpy.init(args=args)
    ros_node = DetectaAruco()  # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
