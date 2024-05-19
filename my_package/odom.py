import rclpy
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile


class Odom:
    def __init__(self):
        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

    def odom_callback(self, data: Odometry):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.yaw = self.euler_from_quaternion(
            [
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.w,
                data.pose.pose.position.z,
            ]
        )[2]
        self.yaw_2pi = (self.yaw + 2 * np.pi) % 2 * np.pi

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    subscriber = Odom()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
