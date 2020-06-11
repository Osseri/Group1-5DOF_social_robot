cmake_minimum_required(VERSION 2.8.3)
project(tool_class)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#Ros
find_package(catkin REQUIRED COMPONENTS
	tf2
	tf2_ros
	tf2_msgs
	tf2_geometry_msgs
	tf_conversions
	sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tool_class
  CATKIN_DEPENDS	 tf2 tf2_ros tf2_msgs tf2_geometry_msgs tf_conversions sensor_msgs
#  DEPENDS system_lib	
)

include_directories(
	include	
	/usr/include/eigen3
	${catkin_INCLUDE_DIRS}	
)

link_directories(	  		 
	/opt/ros/jade/lib	
	lib/linux64	
)

add_library(tool_class src/tool_class.cpp)

