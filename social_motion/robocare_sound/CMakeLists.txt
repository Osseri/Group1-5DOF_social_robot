cmake_minimum_required(VERSION 2.8.3)
project(robocare_sound)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robocare_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES silbot3_sound
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install( 
  PROGRAMS scripts/sound_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
