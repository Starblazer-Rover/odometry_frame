import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import math

from std_msgs.msg import Float32MultiArray, Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from message_filters import Subscriber, ApproximateTimeSynchronizer


class PointPublisher(Node):
    
    def __init__(self):
        super().__init__('point_publisher')

        self.TARGET_LAT= 38.42512274
        self.TARGET_LONG=-110.785219
        
        self.gps_subscriber = Subscriber(self, Float32MultiArray, '/odom/gps')

        self.odom_subscriber = Subscriber(self, Odometry, '/odom')

        self.point_publisher = self.create_publisher(PoseStamped, '/odom/target_point', 10)

        self.approx_sync = ApproximateTimeSynchronizer([self.gps_subscriber, self.odom_subscriber], 1, 20000, allow_headerless=True)

        self.approx_sync.registerCallback(self.listener_callback)
        print('here')

    def create_header(self, frame_id):
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

    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371*39370.1 # Earth radius in kilometers

        # Convert latitude and longitude from degrees to radians
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        # Calculate differences
        dlat = lat2 - lat1
        dlon = lon2 - lon1

        # Haversine formula
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        # Calculate bearing
        angle = math.atan2(math.sin(lon2 - lon1) * math.cos(lat2),
                        math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1))
        angle = math.degrees(angle)
        angle = ((angle + 360) % 360)  # Convert bearing to a compass angle

        return distance, angle

    def listener_callback(self, gps_msg, odometry_msg):
        point_msg = PoseStamped()

        point_msg.header = self.create_header('odom')

        lat = gps_msg.data[0]
        long = gps_msg.data[1]

        distance, angle = self.haversine(lat, long, self.TARGET_LAT, self.TARGET_LONG)
        angle *= -1

        angle += 90

        angle = angle % 360
        
        x = odometry_msg.pose.pose.position.x
        y = odometry_msg.pose.pose.position.y

        print(f'Distance: {distance}, Angle: {angle}')

        point_x = x + distance * math.cos(angle/180 * math.pi)
        point_y = y + distance * math.sin(angle/180 * math.pi)

        point_msg.pose.position.x = point_x
        point_msg.pose.position.y = point_y
        point_msg.pose.position.z = 0.0
        point_msg.pose.orientation.w = 1.0
        point_msg.pose.orientation.x = 0.0
        point_msg.pose.orientation.y = 0.0
        point_msg.pose.orientation.z = 0.0

        self.point_publisher.publish(point_msg)

def main(args=None):
    rclpy.init(args=args)

    point_publisher = PointPublisher()

    try:
        rclpy.spin(point_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        point_publisher.destroy_node()

if __name__ == '__main__':
    main()
