from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

class OdometryFramePublisher(Node):

    def __init__(self):
        super().__init__('odometry_frame_publisher')

        self.__odom_broadcaster = TransformBroadcaster(self)

        self.__odom_subscriber = self.create_subscription(Odometry, '/odom/Odometry', self.listener_callback, 1)
        self.__odom_subscriber

    def listener_callback(self, msg):
        transform = TransformStamped()

        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        self.__odom_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    odometry_frame_publisher = OdometryFramePublisher()

    try:
        rclpy.spin(odometry_frame_publisher)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()