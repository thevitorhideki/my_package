#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import ReliabilityPolicy, QoSProfile
from cv_bridge import CvBridge


class MobileNetDetector(Node):
    """Classe para detecção de objetos com o modelo MobileNetSSD.
    """

    def __init__(self,
                 CONFIDENCE=0.7,
                 args_prototxt="/home/thevitorhideki/colcon_ws/src/my_package/my_package/config/MobileNetSSD_deploy.prototxt.txt",
                 args_model="/home/thevitorhideki/colcon_ws/src/my_package/my_package/config/MobileNetSSD_deploy.caffemodel"
                 ):
        super().__init__('module_net')
        self.CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair",
                        "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
        self.CONFIDENCE = CONFIDENCE
        self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))

        self.args_prototxt = args_prototxt
        self.args_model = args_model
        self.net = self.load_mobilenet()

        self.draw = True
        self.bridge = CvBridge()

        self.subcomp = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

    def image_callback(self, msg: np.ndarray):
        cv_image = self.bridge.imgmsg_to_cv2(
            msg, "bgr8")  # if CompressedImage

        img, results = self.detect(cv_image)

        print(results)
        cv2.imshow("imagem", img)
        cv2.waitKey(1)

    def load_mobilenet(self):
        """Carrega o modelo MobileNetSSD.
        Certifique-se de que os arquivos .prototxt.txt e .caffemodel diretório correto.

        Returns:
            net: modelo carregado
        """
        net = cv2.dnn.readNetFromCaffe(self.args_prototxt, self.args_model)
        return net

    def detect(self, frame: np.ndarray):
        """Detecta objetos na imagem de entrada.
        Filtra as detecções com uma confiança menor que self.CONFIDENCE.

        Args:
            frame (np.ndarray): Imagem de entrada

        Returns:
            image (np.ndarray): Imagem de saida - as detecções são desenhadas apenas se "self.draw = True"
            results ( list(dict) ): Lista de dicionários com as detecções (classe, confidence, bbox(x1, y1, x2, y2))
        """
        image = frame.copy()
        h, w, _ = image.shape
        blob = cv2.dnn.blobFromImage(cv2.resize(
            image, (300, 300)), 0.007843, (300, 300), 127.5)
        self.net.setInput(blob)
        detections = self.net.forward()

        results = []
        for i in np.arange(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]

            if confidence > self.CONFIDENCE:
                idx = int(detections[0, 0, i, 1])
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                label = "{}: {:.2f}%".format(self.CLASSES[idx], confidence)

                if self.draw:
                    cv2.rectangle(image, (startX, startY),
                                  (endX, endY), self.COLORS[idx], 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(image, label, (startX, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS[idx], 2)

                results.append({'classe': self.CLASSES[idx], 'confidence': confidence, 'bbox': (
                    startX, startY, endX, endY)})

        return image, results


def main(args=None):
    rclpy.init(args=args)
    ros_node = MobileNetDetector()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
