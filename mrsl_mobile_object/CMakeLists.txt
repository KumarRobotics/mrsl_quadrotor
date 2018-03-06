cmake_minimum_required(VERSION 2.8.12)
project(mrsl_mobile_object)

find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros quadrotor_msgs gazebo_dev)

find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS roscpp gazebo_ros quadrotor_msgs)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(mrsl_mobile_object SHARED src/mobile_object.cpp)
target_link_libraries(mrsl_mobile_object ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(mrsl_mobile_object ${catkin_EXPORTED_TARGETS})
