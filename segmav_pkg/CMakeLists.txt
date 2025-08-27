cmake_minimum_required(VERSION 3.0.2)
project(segmav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  src/scan_to_obstacle_distance.py
  src/mavsegmav_merged.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
