cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME path_tracker_process)
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


set(PATHTRACKERPROCESS_SOURCE_DIR
        src/source)

set(PATHTRACKERPROCESS_INCLUDE_DIR
        src/include)

set(PATHTRACKERPROCESS_TEST_DIR
        src/test
)

set(PATHTRACKERPROCESS_HEADER_FILES
        ${PATHTRACKERPROCESS_INCLUDE_DIR}/path_tracker_process.h
)

set(PATHTRACKERPROCESS_SOURCE_FILES
        ${PATHTRACKERPROCESS_SOURCE_DIR}/path_tracker_process.cpp
        ${PATHTRACKERPROCESS_SOURCE_DIR}/path_tracker_process_main.cpp

)


find_package(catkin REQUIRED COMPONENTS roscpp robot_process geometry_msgs lib_cvgutils)

catkin_package(
        INCLUDE_DIRS ${PATHTRACKERPROCESS_INCLUDE_DIR}
        LIBRARIES path_tracker_process
        CATKIN_DEPENDS roscpp robot_process
        DEPENDS yaml-cpp
  )


include_directories(${PATHTRACKERPROCESS_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})


add_library(path_tracker_process ${PATHTRACKERPROCESS_SOURCE_FILES} ${PATHTRACKERPROCESS_HEADER_FILES})
add_dependencies(path_tracker_process ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_tracker_process ${catkin_LIBRARIES})


add_executable(path_tracker_process_main ${PATHTRACKERPROCESS_SOURCE_DIR}/path_tracker_process_main.cpp)
add_dependencies(path_tracker_process_main ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_tracker_process_main path_tracker_process)
target_link_libraries(path_tracker_process_main ${catkin_LIBRARIES}) 
target_link_libraries(path_tracker_process yaml-cpp)


#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(path_tracker_test ${PATHTRACKERPROCESS_TEST_DIR}/path_tracker_tester.cpp)
  target_link_libraries(path_tracker_test ${catkin_LIBRARIES})
endif()