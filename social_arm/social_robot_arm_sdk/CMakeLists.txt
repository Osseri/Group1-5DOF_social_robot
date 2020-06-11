################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(social_robot_arm_sdk)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_math
  robotis_controller
  robotis_controller_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(orocos_kdl)
find_package(kdl_parser)

################################################################################
# Declare ROS messages, services and actions
################################################################################

add_message_files(
  FILES
  JointPose.msg
  KinematicsPose.msg
  PlayMotion.msg
  SetJointDeltaPosition.msg
  SetJointTorque.msg
)

add_service_files(
  FILES
  GetJointPose.srv
  GetKinematicsPose.srv
  SetJointPose.srv
  SetKinematicsPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

################################################################################
# Catkin specific configuration
################################################################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp robotis_math robotis_framework_common std_msgs sensor_msgs geometry_msgs
)

################################################################################
# Build
################################################################################

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(social_robot_arm_sdk
  src/social_robot_arm_sdk.cpp
  src/arm_motion_control_module.cpp
  src/arm_control_module.cpp
  src/motion_state.cpp
  src/social_robot_arm_kinematics.cpp
  src/social_robot_arm_task_control.cpp
)

add_dependencies(social_robot_arm_sdk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(social_robot_arm_sdk
  ${catkin_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

################################################################################
# Install
################################################################################

install(TARGETS social_robot_arm_sdk
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################

