cmake_minimum_required(VERSION 2.8.3)
project(robocare_frame)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf_conversions
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robocare_frame
  CATKIN_DEPENDS roscpp roslib rospy sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_msgs tf2_ros tf_conversions
#  DEPENDS system_lib	
)

include_directories(
	include	
	${catkin_INCLUDE_DIRS}		
)

link_directories(  		 
	/opt/ros/jade/lib		
	lib/linux64
)

add_executable(
	robocare_frame	src/robocare_frame.cpp				
)

target_link_libraries(
	robocare_frame	${catkin_LIBRARIES}					
)

install(
	TARGETS robocare_frame
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)




