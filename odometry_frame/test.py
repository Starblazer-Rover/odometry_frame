import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import numpy as np
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from std_msgs.msg import Header, Float64MultiArray


class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        self.timer = self.create_timer(1/30, self.timer_callback)

        time = Clock().now().to_msg()
        print(time)

        self.time = time.sec + (time.nanosec / 1000000000)

    def timer_callback(self):
        print(self.time)

def main(args=None):
    rclpy.init(args=args)

    odometry_publisher = OdometryPublisher()

    try:
        rclpy.spin(odometry_publisher)
    except KeyboardInterrupt:
        odometry_publisher.destroy_node()


if __name__ == '__main__':
    main()
