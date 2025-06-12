cmake_minimum_required(VERSION 2.8.12)
project(mrsl_quadrotor_simulator)

find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros gazebo_dev kr_mav_msgs)

find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(include)

catkin_package(CATKIN_DEPENDS roscpp gazebo_ros kr_mav_msgs)

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

# ROS2 CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(mrsl_quadrotor_simulator)

# ament & rclcpp
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Gazebo‐Classic harmonic support
# find_package(gazebo_ros REQUIRED)
# find_package(gazebo_dev REQUIRED)

# Your custom messages
find_package(kr_mav_msgs REQUIRED)

# Math libs
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

# Include paths
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}            # from gazebo_dev
)

# ----------------------------------------------------------------------------
# Propulsion plugin: SO3 controller
# ----------------------------------------------------------------------------
add_library(mrsl_quadrotor_propulsion SHARED
  src/quadrotor_propulsion.cpp
  src/Quadrotor.cpp
  src/QuadrotorSO3AttitudeControl.cpp
)
ament_target_dependencies(mrsl_quadrotor_propulsion
  rclcpp
  gazebo_ros                       # for gazebo_ros::gazebo_ros
  kr_mav_msgs
)
target_link_libraries(mrsl_quadrotor_propulsion
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}              # from gazebo_dev
)

# ----------------------------------------------------------------------------
# Propulsion plugin: TRPY controller
# ----------------------------------------------------------------------------
add_library(mrsl_quadrotor_propulsion_trpy SHARED
  src/quadrotor_propulsion_trpy.cpp
  src/Quadrotor.cpp
  src/QuadrotorTRPYControl.cpp
)
ament_target_dependencies(mrsl_quadrotor_propulsion_trpy
  rclcpp
  gazebo_ros
  kr_mav_msgs
)
target_link_libraries(mrsl_quadrotor_propulsion_trpy
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# ----------------------------------------------------------------------------
# Install & export
# ----------------------------------------------------------------------------
install(
  TARGETS
    mrsl_quadrotor_propulsion
    mrsl_quadrotor_propulsion_trpy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(
  rclcpp
  gazebo_ros
  kr_mav_msgs
  Eigen3
  Boost
)
ament_package()