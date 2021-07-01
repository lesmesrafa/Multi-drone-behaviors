cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_pid_controller)

# Change C++ version to 2011. By default ROS uses c++98
add_definitions(-std=c++11)

# Directory definitions
set(QUADROTOR_PID_CONTROLLER_SOURCE_DIR
  src/source
)

set(QUADROTOR_PID_CONTROLLER_INCLUDE_DIR
  src/include
)

# Files declaration
set(QUADROTOR_PID_CONTROLLER_SOURCE_FILES
  ${QUADROTOR_PID_CONTROLLER_SOURCE_DIR}/altitude_controller.cpp
  ${QUADROTOR_PID_CONTROLLER_SOURCE_DIR}/position_controller.cpp
  ${QUADROTOR_PID_CONTROLLER_SOURCE_DIR}/speed_controller.cpp
  ${QUADROTOR_PID_CONTROLLER_SOURCE_DIR}/yaw_controller.cpp
  ${QUADROTOR_PID_CONTROLLER_SOURCE_DIR}/pid_control.cpp
)

set(QUADROTOR_PID_CONTROLLER_HEADER_FILES
  ${QUADROTOR_PID_CONTROLLER_INCLUDE_DIR}/altitude_controller.h
  ${QUADROTOR_PID_CONTROLLER_INCLUDE_DIR}/position_controller.h
  ${QUADROTOR_PID_CONTROLLER_INCLUDE_DIR}/speed_controller.h
  ${QUADROTOR_PID_CONTROLLER_INCLUDE_DIR}/yaw_controller.h
  ${QUADROTOR_PID_CONTROLLER_INCLUDE_DIR}/pid_control.h
)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  aerostack_msgs
  pugixml 
  lib_cvgutils
)

###################################
## catkin specific configuration ##
###################################
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${QUADROTOR_PID_CONTROLLER_INCLUDE_DIR}
  LIBRARIES quadrotor_pid_controller
  CATKIN_DEPENDS
  roscpp
  std_msgs
  aerostack_msgs
  pugixml
  lib_cvgutils
  DEPENDS yaml-cpp
)

###########
## Build ##
###########
include_directories(
  ${QUADROTOR_PID_CONTROLLER_INCLUDE_DIR}
  ${QUADROTOR_PID_CONTROLLER_SOURCE_DIR}
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ Library
add_library(quadrotor_pid_controller ${QUADROTOR_PID_CONTROLLER_SOURCE_FILES} ${QUADROTOR_PID_CONTROLLER_HEADER_FILES})
add_dependencies(quadrotor_pid_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(quadrotor_pid_controller ${catkin_LIBRARIES})
target_link_libraries(quadrotor_pid_controller yaml-cpp)
