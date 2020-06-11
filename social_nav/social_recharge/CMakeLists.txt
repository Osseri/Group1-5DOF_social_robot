cmake_minimum_required(VERSION 2.8.3)
project(social_recharge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  robocare_msgs
)

find_package(OpenCV REQUIRED)
IF (OPENCV_FOUND)
  MESSAGE("-- Found OpenCV version ${OPENCV_VERSION}: ${OPENCV_INCLUDE_DIRS}")
ENDIF (OPENCV_FOUND)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES social_recharge
  CATKIN_DEPENDS roscpp rospy roslib std_msgs sensor_msgs robocare_msgs
  DEPENDS system_lib
)

include_directories(
	include		
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}	
)

link_directories(
  	${OpenCV_LIBRARY_DIRS}
  	/opt/ros/jade/lib		
)

add_executable(
        social_recharge_node	src/social_recharge.cpp
							src/tool_class.cpp				
)

target_link_libraries(
        social_recharge_node	${catkin_LIBRARIES}
							${OpenCV_LIBS}
)

add_executable(
        social_drive_node	src/social_test.cpp
							src/tool_class.cpp				
)

target_link_libraries(
        social_drive_node	${catkin_LIBRARIES}
							${OpenCV_LIBS}
)

add_dependencies(social_recharge_node robocare_msgs_generate_messages_cpp)

install(
        TARGETS social_recharge_node
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
