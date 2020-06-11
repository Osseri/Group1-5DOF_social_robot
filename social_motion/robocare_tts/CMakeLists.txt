cmake_minimum_required(VERSION 2.8.3)
project(robocare_tts)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  robocare_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robocare_tts
  DEPENDS roscpp std_msgs robocare_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)

MESSAGE("==========================================")
MESSAGE(" >> check system architecture! ")
EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
MESSAGE( STATUS "Architecture: ${ARCHITECTURE}" )
MESSAGE("------------------------------------------")

if( ${ARCHITECTURE} STREQUAL "armv7l" )

    link_directories(
      lib/arm-linux-gcc
      ${catkin_LIBRARY_DIRS}
    )
    add_executable(robocare_tts
      src/Exception.cpp
      src/tts_node.cpp
    )

    add_dependencies(robocare_tts robocare_msgs_generate_messages_cpp)

    target_link_libraries(
      robocare_tts
      ${catkin_LIBRARIES}
      m
      pthread
      dl
      pttsE
      pttsE_koKR
      pttsE_dsp
    )

    install(TARGETS robocare_tts
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
elseif(  ${ARCHITECTURE} STREQUAL "x86_64" )

    link_directories(
      lib/x86_64
      ${catkin_LIBRARY_DIRS}
    )
    add_executable(robocare_tts
      src/Exception.cpp
      src/tts_node.cpp
    )

    add_dependencies(robocare_tts robocare_msgs_generate_messages_cpp)

    target_link_libraries(
      robocare_tts
      ${catkin_LIBRARIES}
      m
      pthread
      dl
      PowerTTS_M
      MVoice
      EmVoice
      CmVoice
      Parsing
      common
      KMorph
      pthread
      hasp_linux_x86_64_92843
    )

    install(TARGETS robocare_tts
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
else()
    MESSAGE("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
    MESSAGE(" >> SKIP build tts node!")
    MESSAGE("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
endif()
MESSAGE("==========================================")
