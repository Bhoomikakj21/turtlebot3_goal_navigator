cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_goal_navigator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  src/goal_nav_basic.py
  src/goal_nav_with_avoidance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
