cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME quadrotor_pid_controller_process)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)
#add_definitions(-std=c++0x)
#add_definitions(-std=c++03)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries


set(QUADROTORCONTROLLERPROCESS_SOURCE_DIR
        src/source)

set(QUADROTORCONTROLLERPROCESS_INCLUDE_DIR
        src/include)

set(QUADROTORCONTROLLERPROCESS_TEST_DIR
        src/test
)

set(QUADROTORCONTROLLERPROCESS_HEADER_FILES
        ${QUADROTORCONTROLLERPROCESS_INCLUDE_DIR}/quadrotor_pid_controller_process.h
)

set(QUADROTORCONTROLLERPROCESS_SOURCE_FILES
        ${QUADROTORCONTROLLERPROCESS_SOURCE_DIR}/quadrotor_pid_controller_process.cpp
        ${QUADROTORCONTROLLERPROCESS_SOURCE_DIR}/quadrotor_pid_controller_process_main.cpp

)


find_package(catkin REQUIRED COMPONENTS roscpp robot_process geometry_msgs quadrotor_pid_controller aerostack_msgs)

catkin_package(
        INCLUDE_DIRS ${QUADROTORCONTROLLERPROCESS_INCLUDE_DIR}
        LIBRARIES quadrotor_pid_controller_process
        CATKIN_DEPENDS roscpp robot_process quadrotor_pid_controller
  )


include_directories(${QUADROTORCONTROLLERPROCESS_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})


add_library(quadrotor_pid_controller_process ${QUADROTORCONTROLLERPROCESS_SOURCE_FILES} ${QUADROTORCONTROLLERPROCESS_HEADER_FILES})
add_dependencies(quadrotor_pid_controller_process ${catkin_EXPORTED_TARGETS})
target_link_libraries(quadrotor_pid_controller_process ${catkin_LIBRARIES})


add_executable(quadrotor_pid_controller_process_main ${QUADROTORCONTROLLERPROCESS_SOURCE_DIR}/quadrotor_pid_controller_process_main.cpp)
add_dependencies(quadrotor_pid_controller_process_main ${catkin_EXPORTED_TARGETS})
target_link_libraries(quadrotor_pid_controller_process_main quadrotor_pid_controller_process)
target_link_libraries(quadrotor_pid_controller_process_main ${catkin_LIBRARIES})


#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(quadrotor_pid_controller_test ${QUADROTORCONTROLLERPROCESS_TEST_DIR}/new_quadrotor_pid_controller_tester.cpp)
  target_link_libraries(quadrotor_pid_controller_test ${catkin_LIBRARIES})
endif()