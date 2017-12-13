cmake_minimum_required(VERSION 2.8.3)
project(mrsl_quadrotor_simulator)

find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros cmake_modules quadrotor_msgs)

find_package(Boost REQUIRED)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

include_directories(include)

catkin_package(CATKIN_DEPENDS roscpp gazebo_ros)

add_library(mrsl_quadrotor_propulsion SHARED src/quadrotor_propulsion.cpp
  src/Quadrotor.cpp
  src/QuadrotorSO3AttitudeControl.cpp)
target_link_libraries(mrsl_quadrotor_propulsion ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(mrsl_quadrotor_propulsion ${catkin_EXPORTED_TARGETS})
