cmake_minimum_required(VERSION 2.8.3)
project(leap_motion)

find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs geometry_msgs message_generation sensor_msgs visualization_msgs camera_info_manager rospack camera_calibration_parsers)

find_package(OpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")


add_message_files(
    FILES
    leap.msg
    leapros.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   visualization_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES leap_motion
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  if($ENV{LEAP_SDK} $ENV{LEAP_SDK}/include)
)

install(PROGRAMS
  scripts/leap_interface.py scripts/sender.py scripts/subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY
  launch
  camera_info
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
if(DEFINED ENV{LEAP_SDK})
  add_executable(leap_camera src/leap_camera.cpp)
  target_link_libraries(leap_camera
    ${catkin_LIBRARIES} $ENV{LEAP_SDK}/lib/x64/libLeap.so
  )

  add_executable(leap_hands src/leap_hands.cpp)
  target_link_libraries(leap_hands
    ${catkin_LIBRARIES} $ENV{LEAP_SDK}/lib/x64/libLeap.so
  )

  install(TARGETS leap_camera leap_hands
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_leap_motion.test test/test_sender.test)
endif()
