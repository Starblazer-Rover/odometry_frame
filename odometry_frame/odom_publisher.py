import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Header


class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        self.__encoder_subscriber = self.create_subscription(Imu, '/odom/Encoder', self.listener_callback, 1)
        self.__encoder_subscriber

        self.__odometry_publisher = self.create_publisher(Odometry, '/odom/Odometry', 1)

        self.odom = Odometry()
        self.odom.header = self.__create_header('odom')
        self.odom.child_frame_id = "base_link"
        self.__initialize_orientation()
        self.__initialize_pose()

        self.yaw = 0


    def __initialize_orientation(self):
        orientation = Quaternion()

        orientation.w = 1
        orientation.x = 0
        orientation.y = 0
        orientation.z = 0

        self.odom.pose.pose.orientation = orientation

    def __initialize_pose(self):
        pose = Pose()

        pose.x = 0
        pose.y = 0
        pose.z = 0

        self.odom.pose.pose.position = pose

    def __create_header(self, frame_id):
        """Creates a header object for the message

        Header:
            stamp: Time message which has the seconds and nanoseconds since the epoch
            frame_id: TF which the header is relevant to

        Args:
            frame_id (String): This is the transform which the message applies to

        Returns:
            Header: Header containing the timestamp and given frame_id
        """

        # Creates a timer for timestamps
        timer = Clock().now()
        header = Header()

        header.stamp = timer.to_msg()
        header.frame_id = frame_id

        return header
    
    def __calculate_difference(self, d_left, d_right):
        ENCODER_ERROR = 0.5
        WHEEL_DISTANCE = 16 #inches

        difference = d_left - d_right

        if abs(difference) < ENCODER_ERROR * 2:
            delta_x = ((d_left + d_right) / 2) * np.cos(self.yaw)
            delta_y = ((d_left + d_right) / 2) * np.sin(self.yaw)
            delta_theta = 0

        else:
            delta_theta = (d_right - d_left) / WHEEL_DISTANCE

            r_right = d_right / delta_theta
            r_left = d_left / delta_theta

            r_center = (r_right + r_left) / 2

            delta_x = r_center * (np.cos(delta_theta + self.yaw - np.pi/2) - np.cos(self.yaw - np.pi/2))
            delta_y = r_center * (np.sin(delta_theta + self.yaw - np.pi/2) - np.sin(self.yaw - np.pi/2))

        return delta_x, delta_y, delta_theta
    
    def __update_orientation(self):
        self.odom.pose.pose.orientation.z = np.sin(self.yaw / 2)
        self.odom.pose.pose.orientation.w = np.cos(self.yaw / 2)

    def listener_callback(self, msg):

        left = msg.data[0]
        right = msg.data[1]

        deltax, deltay, deltatheta = self.__calculate_difference(left, right)

        self.odom.pose.pose.position.x += deltax
        self.odom.pose.pose.position.y += deltay
        self.yaw += deltatheta

        self.__update_orientation()


def main(args=None):
    rclpy.init(args=args)

    odometry_publisher = OdometryPublisher()

    rclpy.spin(odometry_publisher)

    odometry_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
