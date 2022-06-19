cmake_minimum_required(VERSION 3.0.2)
project(antenna_tracker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  message_generation
)


add_service_files(
  FILES
  SetCoordinates.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
