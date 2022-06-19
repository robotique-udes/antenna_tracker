# antenna_tracker
This ROS package provides the angle an antenna should point to in order to aim directly at a mobile robot. The tracking is based on the robot's and the antenna's GNSS position. The orientation is given in radians in a circle going from -pi to pi, with 0 rad pointing east. The positive direction is counter-clockwise and the negative direction is clockwise.

## Subscribed topics
* fix (sensor_msgs/NavSatFix): GNSS fix of the robot.

## Published topics
* antenna_goal_orientation (std_msgs/Float32): Goal orientation of the antenna to aim directly at the robot.

## Services
* antenna_tracker/set_coordinates (antenna_tracker/SetCoordinates): Set the coordinates of the position of the antenna in the WGS84 format. This can also be done as parameters in the launch file.

## Parameters
* ~latitude (float, default: 0): Latitude of the antenna. Can also be set during runtime with the service.
* ~longitude (float, default: 0): Longitude of the antenna. Can also be set during runtime with the service.
