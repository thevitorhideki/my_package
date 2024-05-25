from my_package.module_aruco import Aruco3d
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2
import numpy as np
import time

# Adicione aqui os imports necessários


class DetectaCreeper(Node, Aruco3d):
    def __init__(self) -> None:
        Node.__init__(self, "creepers_node")
        Aruco3d.__init__(self)

        # Configura o kernel
        self.bridge = CvBridge()
        self.kernel = np.ones((5, 5), np.uint8)

        # Máscaras
        self.filters = {
            "verde": {
                "lower": np.array([40, 120, 120]),
                "upper": np.array([60, 255, 255]),
            },
            "azul": {
                "lower": np.array([100, 120, 120]),
                "upper": np.array([130, 255, 255]),
            },
        }

        # Variáveis
        self.string = String()
        self.string.data = ""
        self.running = True

        ###### Subscribers ######
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        # Coloque aqui os subscribers

        ###### Publishers ######
        self.creeper_pub = self.create_publisher(String, "/creeper", 10)
        # Coloque aqui os publishers

    def flag_callback(self, msg: String) -> None:
        self.running = bool(msg.data)

    def image_callback(self, msg: np.ndarray) -> None:
        # Processa a imagem
        if self.running:
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, "bgr8")  # Raw image
            # cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # Compressed

            bgr, aruco = self.run(cv_image)

            if aruco:
                print(f'Cor: {aruco["color"]}, ID: {aruco["id"][0]}')
                self.string.data = f'{aruco["body_center"][0]} {aruco["color"]} {aruco["id"][0]}'
                self.creeper_pub.publish(self.string)

            cv2.imshow("Identificador de creeper", bgr)
            cv2.waitKey(1)
        else:
            print("Não está rodando")

    def run(self, bgr: np.ndarray) -> list[np.ndarray, dict]:
        # Chame a função self.detect_aruco e armazene os resultados em uma variável.
        _, results = self.detectaAruco(bgr)

        # Essa função retorna uma tupla com duas variáveis. A primeira é uma imagem e a
        # segunda é uma lista de dicionários.
        # Cada dicionário contém as informações de um aruco.

        # Se tiver creeper na tela, chama a função find_creeper e salva seu centro de massa e sua cor em uma lista.
        creepers = []
        creepers += self.find_creeper(bgr, "verde")
        creepers += self.find_creeper(bgr, "azul")

        # Desenvolva a função `match_aruco` para combinar os marcadores Aruco com os corpos dos creepers.
        bgr, matched_pairs = self.match_aruco(bgr, creepers, results)

        if matched_pairs:
            for pair in matched_pairs:
                bgr = self.drawAruco(bgr, pair)

            # Classifica o Aruco por distância
            ranked_aruco = min(matched_pairs, key=lambda x: x["distancia"])

            return [bgr, ranked_aruco]
        return [bgr, None]

    def distance(self, x1, x2) -> int:
        return abs(x1 - x2)

    def match_aruco(self, bgr: np.ndarray, creepers: list, arucos: list) -> tuple[np.ndarray, list[dict]]:
        matched_pairs = []

        for creeper in creepers:
            # Use a função min para ordenar os resultados por distância com base na função self.distance
            if arucos:
                closest = min(
                    arucos, key=lambda x: self.distance(
                        x["centro"][0], creeper[0][0])
                )

                # Remove da lista `results` o aruco mais próximo do corpo `creeper` para evitar que ele seja combinado novamente
                arucos = [
                    aruco
                    for aruco in arucos
                    if aruco["distancia"] != closest["distancia"]
                ]

                # Adiciona na variável o centro do creeper mais próximo na chave "body_center" do dicionário `closest`
                closest["body_center"] = creeper[0]  # (x,y)
                # Adiciona a cor do creeper mais próximo na chave "color" do dicionário `closest`
                closest["color"] = creeper[1]

                # Desenha uma linha entre o centro do creeper e o centro do marcador Aruco
                cv2.line(
                    bgr,
                    tuple(closest["centro"]),
                    tuple(closest["body_center"]),
                    (0, 0, 255),
                    2,
                )

                # Adiciona o par combinado na lista `matched_pairs`
                matched_pairs.append(closest)

        return bgr, matched_pairs

    def find_creeper(self, bgr: np.ndarray, color: str) -> list:
        creepers = []
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv, self.filters[color]["lower"], self.filters[color]["upper"]
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:
                x, y, w, h = cv2.boundingRect(contour)
                cx, cy = x + w // 2, y + h // 2
                creepers.append([(cx, cy), color])

        return creepers


def main(args=None) -> None:
    rclpy.init(args=args)
    ros_node = DetectaCreeper()  # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
