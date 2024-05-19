from std_msgs.msg import Float64
import numpy as np
import time


class Garra():  # Mude o nome da classe

    def __init__(self):
        # Inicialização de variáveis

        self.delay = 1.0
        self.msg = Float64()

        # Publishers
        self.ombro = self.create_publisher(
            Float64,
            "/joint1_position_controller/command",
            10
        )
        self.garra = self.create_publisher(
            Float64,
            "/joint2_position_controller/command",
            10
        )

    def controla_garra(self, command: str):
        if command == 'abre':
            self.msg.data = -1.0
            self.garra.publish(self.msg)
        elif command == 'fecha':
            self.msg.data = 0.0
            self.garra.publish(self.msg)
        elif command == 'cima':
            self.msg.data = 1.5
            self.ombro.publish(self.msg)
        elif command == 'frente':
            self.msg.data = 0.0
            self.ombro.publish(self.msg)
        elif command == 'baixo':
            self.msg.data = -1.0
            self.ombro.publish(self.msg)

        time.sleep(self.delay)
