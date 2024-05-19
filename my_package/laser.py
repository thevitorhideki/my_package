import numpy as np
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile


class Laser:
    def __init__(self):
        self.front = [0]
        self.opening = 10
        self.create_subscription(
            LaserScan,
            "/scan",
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    def laser_callback(self, msg: LaserScan):
        self.laser_msg = np.array(msg.ranges).round(decimals=2)
        self.laser_msg[self.laser_msg == 0] = np.inf
        self.laser_msg = list(self.laser_msg)

        self.front = self.laser_msg[: self.opening] + self.laser_msg[-self.opening :]
        self.left = self.laser_msg[90 - self.opening : 90 + self.opening]
        self.right = self.laser_msg[275 - self.opening : 275 + self.opening]
        self.back = self.laser_msg[180 - self.opening : 180 + self.opening]

        self.custom_laser()

    def custom_laser(self):
        pass
