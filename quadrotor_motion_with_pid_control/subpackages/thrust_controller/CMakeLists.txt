cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME thrust_controller)
project(${PROJECT_NAME})

# Change C++ version to 2011. By default ROS uses c++98
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  mavros_msgs
  mav_msgs
  robot_process
  tf
  ros_utils_lib
)

catkin_package(
  INCLUDE_DIRS src/include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs mavros_msgs robot_process mav_msgs tf ros_utils_lib yaml-cpp
  )

include_directories(
  src/include
  ${catkin_INCLUDE_DIRS}
)

set(CPP_PROJECT_FILES 
  src/source/thrust_controller.cpp
)

set(HEADER_PROJECT_FILES
  src/include/thrust_controller.hpp
  
)

add_library(${PROJECT_NAME}
  ${CPP_PROJECT_FILES}
  ${HEADER_PROJECT_FILES}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} yaml-cpp)

add_executable(${PROJECT_NAME}_node src/source/thrust_controller_node.cpp  ) 
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})
