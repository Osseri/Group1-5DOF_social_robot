cmake_minimum_required(VERSION 2.8.3)
project(social_avatar)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY system  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
install(DIRECTORY libs  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
install(DIRECTORY scripts  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
install(DIRECTORY avatar  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

catkin_install_python (
        PROGRAMS        scripts/social_avatar
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

