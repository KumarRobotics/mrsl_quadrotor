cmake_minimum_required(VERSION 2.8.3)
project(mrsl_mobile_object)

find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros cmake_modules quadrotor_msgs)

find_package(Boost REQUIRED)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS roscpp gazebo_ros)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

#include_directories(include)

add_library(mrsl_mobile_object SHARED src/mobile_object.cpp)
target_link_libraries(mrsl_mobile_object ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(mrsl_mobile_object ${catkin_EXPORTED_TARGETS})

