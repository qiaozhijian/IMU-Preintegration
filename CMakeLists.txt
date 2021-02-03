cmake_minimum_required(VERSION 2.8)
project(IMU_Preintegration)

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -std=c++11 -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
        rospy
        rosbag
        roscpp
        message_filters
        cv_bridge
        tf
        )

set(ALL_TARGET_LIBRARIES "")
include(cmake/boost.cmake)
include(cmake/eigen.cmake)
include(cmake/OpenCV.cmake)
include(cmake/pangolin.cmake)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE ALL_SRCS "src/*.cpp" "src/*.cc")
message(ALL_SRCS ${ALL_SRCS})

add_executable(ros_stereo_odo_serial_node3 Examples/ros_stereo_odo_serial.cc ${ALL_SRCS})
target_link_libraries(ros_stereo_odo_serial_node3 ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(ros_stereo_odo_node Examples/ros_stereo_odo.cc ${ALL_SRCS})
target_link_libraries(ros_stereo_odo_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})
