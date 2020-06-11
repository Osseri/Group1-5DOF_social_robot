cmake_minimum_required(VERSION 2.8.3)
project(robocare_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation)

add_message_files(
  DIRECTORY msg
  FILES
    TTSSetProperties.msg
    TTSViseme.msg
    NavigationStatus.msg
)

add_service_files(
  DIRECTORY srv
  FILES
	TTSMake.srv
  	SoundPlay.srv
	ChangeMap.srv
    	DockOnStation.srv
    	LeaveStation.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp rospy message_runtime std_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)
