import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile


class SecondNode(Node):

    def __init__(self):
        super().__init__("second_node")
        self.timer = self.create_timer(0.25, self.control)
        self.x, self.y = False

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

    def odom_callback(self, data: Odometry):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

    def control(self):
        print(f"Posição x: {self.x}")
        print(f"Posição y: {self.y}\n")


def main(args=None):
    rclpy.init(args=args)
    second_node = SecondNode()

    rclpy.spin(second_node)

    second_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
