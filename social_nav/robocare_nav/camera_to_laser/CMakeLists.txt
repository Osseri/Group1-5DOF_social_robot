cmake_minimum_required(VERSION 2.8.3)
project(camera_to_laser)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#OpenGL
find_package(OpenGL REQUIRED)
#find_package(GLUT REQUIRED)
#set(GLFW_INCLUDE_PATH "" CACHE PATH "The directory that contains GL/glfw.h" )
#set(GLFW_LIBRARY "" CACHE FILEPATH "libglfw.a or glfw.lib" )
#set(GLEW_INCLUDE_PATH "" CACHE PATH "The directory that contains GL/glew.h" )
#set(GLEW_LIBRARY "" CACHE FILEPATH "libglew.a or glew.lib" )
#set(GLM_INCLUDE_PATH "" CACHE PATH "The directory that contains glm/glm.hpp" )

#Ros
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES camera_to_laser
  CATKIN_DEPENDS roscpp roslib rospy sensor_msgs std_msgs pcl_conversions pcl_ros
#  DEPENDS system_lib	
)

include_directories(
	include	
	${catkin_INCLUDE_DIRS}		
	${OPENGL_INCLUDE_DIRS}
	${GLUT_INCLUDE_DIRS}	
)

link_directories(  		 
	/opt/ros/jade/lib		
	lib/linux64
)

add_executable(
	camera_to_laser		src/camera_to_laser.cpp		
							src/nui_tracker.cpp
							src/pcl_tools.cpp								
							src/tool_class.cpp											
)

target_link_libraries(
	camera_to_laser		libnuitrack.so
							libmiddleware.so											
							${catkin_LIBRARIES}					
)

install(
	TARGETS camera_to_laser
			RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
			ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
			LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install( FILES lib/linux64/libnuitrack.so
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}    
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
  PATTERN ".svn" EXCLUDE
)


