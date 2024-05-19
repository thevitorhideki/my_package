import rclpy
from rclpy.node import Node
from my_package.laser import Laser


class TestLaser(Node, Laser):
    def __init__(self):
        Node.__init__(self, "test_laser_node")
        Laser.__init__(self)
        self.timer = self.create_timer(1, self.control)

    def control(self):
        print(
            {
                "Frente": self.front,
                "Esquerda": self.left,
                "Direita": self.right,
                "Atrás": self.back,
            }
        )


def main(args=None):
    rclpy.init(args=args)
    ros_node = TestLaser()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
