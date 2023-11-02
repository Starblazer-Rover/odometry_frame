import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Header


class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        self.default_covariance = np.zeros(36)

        # Set the first imu to None so the subscriber knows if its the first msg of imu
        self.first_imu = None

        self.__imu_subscriber = self.create_subscription(Imu, '/odom/Imu', self.listener_callback, 1)
        self.__imu_subscriber

        self.__odometry_publisher = self.create_publisher(Odometry, '/odom/Odometry', 1)

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
    
    def __calculate_duration(self, msg):
        """Takes the first time from imu and subtracts it from the second time

        Args:
            msg (Imu Message): Input from /camera/Imu

        Returns:
            _type_: Returns the difference of the two times
        """
        duration_sec = msg.header.stamp.sec - self.first_imu.header.stamp.sec
        duration_nanosec = msg.header.stamp.nanosec - self.first_imu.header.stamp.nanosec

        duration = duration_sec + (duration_nanosec / 1000000000)

        return duration
    
    def __initialize_pose(self):
        """
        Sets all the Pose values to 0 because the calculations use previous values and they cannot be None
        """
        self.pose = Pose()
        self.pose.position.x = 0.0
        self.pose.position.y = 0.0
        self.pose.position.z = 0.0
    
    def __calculate_pose(self, msg):
        """Calculates pose from previously known velocity and acceleration using kinematics

        Args:
            msg (Imu msg): Input from /camera/Imu
        """

        # pose = vt + 1/2at
        self.pose.position.x += (self.twist.linear.x * self.duration) + (0.5 * msg.linear_acceleration.x * (self.duration**2))
        self.pose.position.y += (self.twist.linear.y * self.duration) + (0.5 * msg.linear_acceleration.y * (self.duration**2))
        self.pose.position.z += (self.twist.linear.z * self.duration) + (0.5 * msg.linear_acceleration.z * (self.duration**2)) 

        self.pose.orientation = msg.orientation

    def __initialize_twist(self):
        """
        Initializes twist values as 0 because when accessing them for the first time, they cannot be None
        """
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

    def __calculate_twist(self, msg):
        """Calculates twist from inputted acceleration of msg

        Args:
            msg (Imu Message): Input from /camera/Imu
        """

        # velocity = at
        self.twist.linear.x += msg.linear_acceleration.x * self.duration
        self.twist.linear.y += msg.linear_acceleration.y * self.duration
        self.twist.linear.z += msg.linear_acceleration.z * self.duration

        self.twist.angular = msg.angular_velocity

    def __accel_to_zero(self, msg):
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 0.0
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0

        return msg

    def __vector_comparison(self, vector):
        return (vector.x == 0 and vector.y == 0 and vector.z == 0)
        

    def __bias_odom(self, msg):
        #if self.__vector_comparison(msg.linear_acceleration) and self.__vector_comparison(msg.angular_velocity):
        if False:
            self.__initialize_twist()
            msg = self.__accel_to_zero(msg)
            return msg
            
            
        else:
            self.get_logger().info(f"{self.twist.linear.x}, {self.twist.linear.y}, {self.twist.linear.x}, {self.duration}")
            return msg
            

    def __create_odom(self):
        """Creates the odometry message using all the previously calculated data

        Returns:
            Odom Message: Keeps track of all position values for the rover
        """
        odom = Odometry()

        odom.pose.pose = self.pose
        odom.pose.covariance = self.default_covariance
        odom.twist.twist = self.twist
        odom.twist.covariance = self.default_covariance
        odom.child_frame_id = 'base_link'
        odom.header = self.__create_header('odom')

        return odom

    def listener_callback(self, msg):
        if self.first_imu == None:
            self.first_imu = msg
            self.__initialize_pose()
            self.__initialize_twist()
            return

        self.duration = self.__calculate_duration(msg)

        msg = self.__bias_odom(msg)

        self.__calculate_pose(msg)

        self.__calculate_twist(msg)

        odom = self.__create_odom()

        self.first_imu = msg

        self.__odometry_publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)

    odometry_publisher = OdometryPublisher()

    rclpy.spin(odometry_publisher)

    odometry_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
