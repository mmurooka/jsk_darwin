cmake_minimum_required(VERSION 2.8.3)
project(jsk_darwin)

find_package(catkin REQUIRED COMPONENTS
  roseus
  roscpp
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_math
  )

catkin_package(
  CATKIN_DEPENDS
  roseus
  roscpp
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_math
  )

include_directories(${catkin_INCLUDE_DIRS})

add_executable(darwin_direct_controller src/darwin_direct_controller)
target_link_libraries(darwin_direct_controller ${catkin_LIBRARIES})
