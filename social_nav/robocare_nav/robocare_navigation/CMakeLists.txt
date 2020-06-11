cmake_minimum_required(VERSION 2.8.3)
project(robocare_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  rospy
  robocare_msgs
  sensor_msgs
  std_msgs
  tf_conversions
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  tool_class
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robocare_navigation
  CATKIN_DEPENDS roslib roscpp rospy std_msgs sensor_msgs robocare_msgs tf2 tf2_geometry_msgs tf2_msgs tf2_ros tf_conversions tool_class
  #DEPENDS system_lib
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
	robocare_navigation	src/robocare_navigation.cpp												
)

target_link_libraries(
	robocare_navigation	${catkin_LIBRARIES} 
							${OpenCV_LIBS}
)

install(
	TARGETS robocare_navigation
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#add_executable(
#	robocare_navigation_test		src/robocare_navigation_test.cpp								
#									src/tool_class.cpp
#									src/Config.cpp
#)

#target_link_libraries(
#	robocare_navigation_test		${catkin_LIBRARIES} 
#									${OpenCV_LIBS}
#)

#install(
#	TARGETS robocare_navigation_test
#			RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#			ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#			LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

add_executable(
	scan_fusion		src/scan_fusion.cpp										
)

target_link_libraries(
	scan_fusion		${catkin_LIBRARIES} 
						${OpenCV_LIBS}
)

install(
	TARGETS scan_fusion
			RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
			ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
			LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#add_dependencies(robocare_navigation robocare_msgs_generate_messages_cpp)
#add_dependencies(robocare_navigation_test robocare_msgs_generate_messages_cpp)

install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
  PATTERN ".svn" EXCLUDE
)


