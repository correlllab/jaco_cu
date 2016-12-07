cmake_minimum_required(VERSION 2.8.3)
project(kinova_control)

# C++ 11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ros_control_boilerplate
  controller_manager
  roscpp
  kinova_driver
  control_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinova_hw_interface
  CATKIN_DEPENDS
    ros_control_boilerplate
    controller_manager
    roscpp
    kinova_driver
    control_msgs
)

###########
## Build ##
###########

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(kinova_control src/kinova_hw_interface.cpp)
target_link_libraries(kinova_control
    ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(kinova_hw_main src/kinova_hw_main.cpp)
target_link_libraries(kinova_hw_main
    kinova_control
    ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(kinova_manual_control src/kinova_manual_control.cpp)
target_link_libraries(kinova_manual_control
    ${catkin_LIBRARIES}
)
