import rclpy
from rclpy.node import Node
from my_package.odom import Odom


class TestOdom(Node, Odom):

    def __init__(self):
        Node.__init__(self, "test_odom_node")
        Odom.__init__(self)
        self.timer = self.create_timer(1, self.control)

    def control(self):
        print([self.x, self.y, self.yaw_2pi])


def main(args=None):
    rclpy.init(args=args)
    ros_node = TestOdom()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
