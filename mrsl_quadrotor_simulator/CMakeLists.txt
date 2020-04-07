cmake_minimum_required(VERSION 2.8.12)
project(mrsl_quadrotor_simulator)

find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros gazebo_dev quadrotor_msgs)

find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(include)

catkin_package(CATKIN_DEPENDS roscpp gazebo_ros quadrotor_msgs)

add_library(mrsl_quadrotor_propulsion SHARED src/quadrotor_propulsion.cpp
  src/Quadrotor.cpp
  src/QuadrotorSO3AttitudeControl.cpp)
target_link_libraries(mrsl_quadrotor_propulsion ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(mrsl_quadrotor_propulsion ${catkin_EXPORTED_TARGETS})

add_library(mrsl_quadrotor_propulsion_trpy SHARED src/quadrotor_propulsion_trpy.cpp
  src/Quadrotor.cpp
  src/QuadrotorTRPYControl.cpp)
target_link_libraries(mrsl_quadrotor_propulsion_trpy ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(mrsl_quadrotor_propulsion_trpy ${catkin_EXPORTED_TARGETS})
