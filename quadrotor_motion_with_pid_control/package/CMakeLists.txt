cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_motion_with_pid_control)

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)


# Directories definition
set(QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR
  src/source
)

set(QUADROTOR_MOTION_WITH_PID_CONTROL_INCLUDE_DIR
  src/include
)

set(QUADROTOR_MOTION_WITH_PID_CONTROL_HEADER_FILES
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_INCLUDE_DIR}/behavior_keep_hovering_with_pid_control.h 
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_INCLUDE_DIR}/behavior_keep_moving_with_pid_control.h 
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_INCLUDE_DIR}/behavior_follow_path.h 
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_INCLUDE_DIR}/behavior_rotate_with_pid_control.h
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_INCLUDE_DIR}/behavior_quadrotor_pid_motion_control.h
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_INCLUDE_DIR}/behavior_take_off_with_pid.h
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_INCLUDE_DIR}/behavior_land_with_pid.h

        ${QUADROTOR_MOTION_WITH_PID_CONTROL_INCLUDE_DIR}/behavior_take_a_look_at_target_with_pid_control.h
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_INCLUDE_DIR}/behavior_reach_target_altitude_with_pid_control.h
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_INCLUDE_DIR}/behavior_follow_target_with_pid_control.h
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_INCLUDE_DIR}/behavior_move_away_from_robot_with_pid_control.h
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_INCLUDE_DIR}/behavior_get_close_to_target_with_pid_control.h
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_INCLUDE_DIR}/behavior_keep_look_at_target_with_pid_control.h
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_INCLUDE_DIR}/behavior_keep_target_altitude_with_pid_control.h
)

set(QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_FILES
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_keep_hovering_with_pid_control.cpp 
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_keep_moving_with_pid_control.cpp 
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_follow_path.cpp 
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_rotate_with_pid_control.cpp
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_quadrotor_pid_motion_control.cpp
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_quadrotor_pid_thrust_control.cpp
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_take_off_with_pid.cpp
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_land_with_pid.cpp

        ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_take_a_look_at_target_with_pid_control.cpp
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_reach_target_altitude_with_pid_control.cpp
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_follow_target_with_pid_control.cpp 
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_move_away_from_robot_with_pid_control.cpp
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_get_close_to_target_with_pid_control.cpp
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_keep_look_at_target_with_pid_control.cpp
        ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_keep_target_altitude_with_pid_control.cpp
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  behavior_execution_manager
  geometry_msgs
  aerostack_msgs
  behavior_execution_manager_msgs
  ros_utils_lib
  tf
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${QUADROTOR_MOTION_WITH_PID_CONTROL_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp
  std_msgs
  behavior_execution_manager
  geometry_msgs
  aerostack_msgs
  behavior_execution_manager_msgs
  ros_utils_lib
  tf
  DEPENDS yaml-cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(BehaviorKeepHoveringWithPidControl ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_keep_hovering_with_pid_control.cpp)
target_link_libraries(BehaviorKeepHoveringWithPidControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorKeepHoveringWithPidControl yaml-cpp)
add_executable(BehaviorKeepMovingWithPidControl ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_keep_moving_with_pid_control.cpp)
target_link_libraries(BehaviorKeepMovingWithPidControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorKeepMovingWithPidControl yaml-cpp)
add_executable(BehaviorFollowPath ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_follow_path.cpp)
target_link_libraries(BehaviorFollowPath ${catkin_LIBRARIES})
target_link_libraries(BehaviorFollowPath yaml-cpp)
add_executable(BehaviorRotateWithPidControl ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_rotate_with_pid_control.cpp)
target_link_libraries(BehaviorRotateWithPidControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorRotateWithPidControl yaml-cpp)
add_executable(BehaviorQuadrotorPidMotionControl ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_quadrotor_pid_motion_control.cpp)
target_link_libraries(BehaviorQuadrotorPidMotionControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorQuadrotorPidMotionControl yaml-cpp)
add_executable(BehaviorQuadrotorPidThrustControl ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_quadrotor_pid_thrust_control.cpp)
target_link_libraries(BehaviorQuadrotorPidThrustControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorQuadrotorPidThrustControl yaml-cpp)
add_executable(BehaviorTakeOffWithPid ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_take_off_with_pid.cpp)
target_link_libraries(BehaviorTakeOffWithPid ${catkin_LIBRARIES})
target_link_libraries(BehaviorTakeOffWithPid yaml-cpp)
add_executable(BehaviorLandWithPid ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_land_with_pid.cpp)
target_link_libraries(BehaviorLandWithPid ${catkin_LIBRARIES})
target_link_libraries(BehaviorLandWithPid yaml-cpp)

add_executable(BehaviorTakeALookAtTargetWithPidControl ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_take_a_look_at_target_with_pid_control.cpp)
target_link_libraries(BehaviorTakeALookAtTargetWithPidControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorTakeALookAtTargetWithPidControl yaml-cpp)
add_executable(BehaviorFollowTargetWithPidControl ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_follow_target_with_pid_control.cpp)
target_link_libraries(BehaviorFollowTargetWithPidControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorFollowTargetWithPidControl yaml-cpp)
add_executable(BehaviorMoveAwayFromRobotWithPidControl ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_move_away_from_robot_with_pid_control.cpp)
target_link_libraries(BehaviorMoveAwayFromRobotWithPidControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorMoveAwayFromRobotWithPidControl yaml-cpp)
add_executable(BehaviorReachTargetAltitudeWithPidControl ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_reach_target_altitude_with_pid_control.cpp)
target_link_libraries(BehaviorReachTargetAltitudeWithPidControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorReachTargetAltitudeWithPidControl yaml-cpp)
add_executable(BehaviorGetCloseToTargetWithPidControl ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_get_close_to_target_with_pid_control.cpp)
target_link_libraries(BehaviorGetCloseToTargetWithPidControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorGetCloseToTargetWithPidControl yaml-cpp)
add_executable(BehaviorKeepLookAtTargetWithPidControl ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_keep_look_at_target_with_pid_control.cpp)
target_link_libraries(BehaviorKeepLookAtTargetWithPidControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorKeepLookAtTargetWithPidControl yaml-cpp)
add_executable(BehaviorKeepTargetAltitudeWithPidControl ${QUADROTOR_MOTION_WITH_PID_CONTROL_SOURCE_DIR}/behavior_keep_target_altitude_with_pid_control.cpp)
target_link_libraries(BehaviorKeepTargetAltitudeWithPidControl ${catkin_LIBRARIES})
target_link_libraries(BehaviorKeepTargetAltitudeWithPidControl yaml-cpp)



if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(BehaviorKeepHoveringWithPidControl ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorKeepMovingWithPidControl ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorFollowPath ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorRotateWithPidControl ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorQuadrotorPidMotionControl ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorQuadrotorPidThrustControl ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorTakeOffWithPid ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorLandWithPid ${catkin_EXPORTED_LIBRARIES})

  add_dependencies(BehaviorTakeALookAtTargetWithPidControl ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorFollowTargetWithPidControl ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorMoveAwayFromRobotWithPidControl ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorReachTargetAltitudeWithPidControl ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorGetCloseToTargetWithPidControl ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorKeepLookAtTargetWithPidControl ${catkin_EXPORTED_LIBRARIES})
  add_dependencies(BehaviorKeepTargetAltitudeWithPidControl ${catkin_EXPORTED_LIBRARIES})
endif()
