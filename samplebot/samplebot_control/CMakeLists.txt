cmake_minimum_required(VERSION 2.8.3)
project(samplebot_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hardware_interface
  transmission_interface
  controller_manager
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
   roscpp
   std_msgs
   hardware_interface
   transmission_interface
   controller_manager
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(samplebot_control src/samplebot_control.cpp src/samplebot_hw.cpp)
target_link_libraries(samplebot_control ${catkin_LIBRARIES})
