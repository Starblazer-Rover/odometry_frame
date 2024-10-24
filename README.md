# Odometry Frame

This repository is designed to handle anything regarding the rovers's position and odometry

## Odometry Scripts

### Odometry:

```bash
ros2 run odometry_frame odom
```

#### Script: odom_publisher.py

#### Subscribed Topics:

/movement/Encoder  
/movement/Imu

#### Subscribed Messages:

std_msgs/Float64MuliArray
std_msgs/Float64

#### Published Topics:

/odom

#### Published Messages:

nav_msgs/Odometry

#### Description:

This code takes Encoder values and Imu values to calculate the change in position of the rover.

### Transform:

```bash
ros2 run odometry_frame transform
```

#### Script: odom_transform.py

#### Subscribed Topics:

/odom

#### Subscribed Messages:

nav_msgs/Odometry

#### Description:

Publishes the odom transform linked to base_link with the Odometry message

### Target:

```bash
ros2 run odometry_frame target
```

#### Script: target_point.py

#### Subscribed Topics:

/odom
/odom/gps

#### Subscribed Messages:

nav_msgs/Odometry
std_msgs/Float32MultiArray

#### Published Topics:

/odom/target_point

#### Published Messages:

geometry_msgs/PoseStamped

#### Description:

Creates a target pose of any GPS location relative to the rover's position