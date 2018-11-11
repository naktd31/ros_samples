cmake_minimum_required(VERSION 2.8.3)
project(cr2016_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  angles
  control_toolbox
  gazebo_ros_control
  controller_manager
  hardware_interface
  joint_limits_interface
  transmission_interface
)

find_package(gazebo REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cr2016_control
  CATKIN_DEPENDS
    roscpp
    angles
    control_toolbox
    gazebo_ros_control
    controller_manager
    hardware_interface
    joint_limits_interface
    transmission_interface
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} src/cr2016_hardware_gazebo.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
