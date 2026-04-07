cmake_minimum_required(VERSION 3.0.2)
project(com760cw2_group5)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

## Custom messages
add_message_files(
  FILES
  RobotStatus.msg
)

## Custom services
add_service_files(
  FILES
  SetGoal.srv
  SwitchBehavior.srv
)

## Generate messages/services with dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs nav_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark Python scripts as installable
catkin_install_python(PROGRAMS
  scripts/GoToPoint.py
  scripts/FollowWall.py
  scripts/bug2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
