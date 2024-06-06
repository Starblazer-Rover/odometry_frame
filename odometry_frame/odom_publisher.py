import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import numpy as np
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Header, Float64MultiArray, Float64

from message_filters import Subscriber, ApproximateTimeSynchronizer


class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        self.first_message = True

        self.__encoder_subscriber = Subscriber(self, Float64MultiArray, '/movement/Encoder')
        self.__encoder_subscriber

        self.__imu_subscriber = Subscriber(self, Float64, '/movement/Imu')

        self.__odometry_publisher = self.create_publisher(Odometry, '/odom', 1)

        self.approx_sync = ApproximateTimeSynchronizer([self.__encoder_subscriber, self.__imu_subscriber], 1, 0.1, allow_headerless=True)

        self.odom = Odometry()
        self.odom.header = self.__create_header('odom')
        self.odom.child_frame_id = "base_link"
        self.__initialize_pose()
        self.__initialize_twist()

        self.yaw = 0
        self.offset = [0, 0]

        time = Clock().now().to_msg()

        self.time = time.sec + (time.nanosec / 1000000000)

        self.approx_sync.registerCallback(self.listener_callback)

    def __initialize_pose(self):
        """
        Covariance:
        [[xx, xy, xz, xr, xp, xq],
         [yx, yy, yz, yr, yp, yq], 
         [zx, zy, zz, zr, zp, zq], 
         [rx, ry, rz, rr, rp, rq], 
         [px, py, pz, pr, pp, pq], 
         [qx, qy, qz, qr, qp, qq]]

        For pose covariance with encoders for differential drive on a 2-D plane, x, y, and q are the only values used.
        x = pose.x
        y = pose.y
        q = orientation.z

        Higher values are higher error
        """
        pose_covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 
                           0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 
                           0.0, 0.0, 0.0, 0.3, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.3]

        
        pose = Pose()

        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0

        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0

        self.odom.pose.covariance = pose_covariance

        self.odom.pose.pose = pose

    def __initialize_twist(self):
        #Change if you are confident in encoder twist values
        twist_covariance = [0.0] * 36

        twist = Twist()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        self.odom.twist.covariance = twist_covariance

        self.odom.twist.twist = twist

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
        ENCODER_ERROR = 0.05
        WHEEL_DISTANCE = 30 #inches

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

    def listener_callback(self, encoder_msg:Float64MultiArray, imu_msg:Float64):

        self.yaw = imu_msg.data

        if self.first_message is True:
            self.offset[0] = encoder_msg.data[0]
            self.offset[1] = encoder_msg.data[1]
            self.first_message = False
            return

        left = (encoder_msg.data[0]) - self.offset[0]
        right = (encoder_msg.data[1]) - self.offset[1]

        print("Left: {:<25}, Right: {:<25}, Yaw: {:>0}".format(left, right, (self.yaw * (180 / math.pi)) % 360))


        self.offset[0] = encoder_msg.data[0]
        self.offset[1] = encoder_msg.data[1]

        deltax, deltay, deltatheta = self.__calculate_difference(left, right)

        time = Clock().now().to_msg()
        time = time.sec + (time.nanosec / 1000000000)

        gap = time - self.time

        self.time = time

        """
        self.odom.twist.twist.linear.x = deltax * gap
        self.odom.twist.twist.linear.y = deltay * gap
        self.odom.twist.twist.angular.z = deltatheta * gap
        """

        self.odom.pose.pose.position.x += deltax
        self.odom.pose.pose.position.y += deltay

        self.__update_orientation()

        self.__odometry_publisher.publish(self.odom)


def main(args=None):
    rclpy.init(args=args)

    odometry_publisher = OdometryPublisher()

    try:
        rclpy.spin(odometry_publisher)
    except KeyboardInterrupt:
        odometry_publisher.destroy_node()


if __name__ == '__main__':
    main()
