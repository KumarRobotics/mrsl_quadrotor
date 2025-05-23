# cmake_minimum_required(VERSION 3.10)
# project(mrsl_quadrotor_utils)
# # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

# ## Find catkin macros and libraries
# ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# ## is used, also find other catkin packages

# find_package(catkin REQUIRED COMPONENTS roscpp tf2_ros topic_tools eigen_conversions tf_conversions)
# find_package(cmake_modules)
# find_package(Eigen3 REQUIRED)
# include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
 
# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp 
#   DEPENDS)

# add_executable(change_header src/change_header.cpp)
# target_link_libraries(change_header ${catkin_LIBRARIES} )
# add_executable(change_frame src/change_frame.cpp)
# target_link_libraries(change_frame ${catkin_LIBRARIES})
# add_executable(change_odom src/change_odom.cpp)
# target_link_libraries(change_odom ${catkin_LIBRARIES})
# add_executable(msg_to_tf src/msg_to_tf.cpp)
# target_link_libraries(msg_to_tf ${catkin_LIBRARIES})
# add_executable(publish_tf src/publish_tf.cpp)
# target_link_libraries(publish_tf ${catkin_LIBRARIES})
# add_executable(publish_horizon src/publish_horizon.cpp)
# target_link_libraries(publish_horizon ${catkin_LIBRARIES})
# add_executable(publish_rotate src/publish_rotate.cpp)
# target_link_libraries(publish_rotate ${catkin_LIBRARIES})
# add_executable(tf_pub src/tf_pub.cpp)
# target_link_libraries(tf_pub ${catkin_LIBRARIES})

#cs_install()
 
#cs_export()

# ROS2 UPDATE FOR CMAKELIST
cmake_minimum_required(VERSION 3.10)
project(mrsl_quadrotor_utils)

# Use the ament build system
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(tf2_ros            REQUIRED)
find_package(tf2_eigen          REQUIRED)
find_package(tf2_eigen_kdl      REQUIRED)
find_package(nav_msgs          REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)

# Include headers
# include_directories(
#   include
# )

# Executables
add_executable(change_header   src/change_header_ros2.cpp)
ament_target_dependencies(change_header
  rclcpp tf2_ros tf2_eigen tf2_eigen_kdl nav_msgs geometry_msgs tf2_geometry_msgs
)

# add_executable(change_frame    src/change_frame.cpp)
# ament_target_dependencies(change_frame
#   rclcpp tf2_ros tf2_eigen tf2_eigen_kdl
# )

add_executable(change_odom     src/change_odom_ros2.cpp)
ament_target_dependencies(change_odom
  rclcpp tf2_ros tf2_eigen tf2_eigen_kdl nav_msgs geometry_msgs tf2_geometry_msgs
)

add_executable(msg_to_tf       src/msg_to_tf_ros2.cpp)
ament_target_dependencies(msg_to_tf
  rclcpp tf2_ros tf2_eigen tf2_eigen_kdl nav_msgs geometry_msgs
)

# add_executable(publish_tf      src/publish_tf.cpp)
# ament_target_dependencies(publish_tf
#   rclcpp tf2_ros tf2_eigen tf2_eigen_kdl
# )

# add_executable(publish_horizon src/publish_horizon.cpp)
# ament_target_dependencies(publish_horizon
#   rclcpp tf2_ros tf2_eigen tf2_eigen_kdl
# )

# add_executable(publish_rotate  src/publish_rotate.cpp)
# ament_target_dependencies(publish_rotate
#   rclcpp tf2_ros tf2_eigen tf2_eigen_kdl
# )

# add_executable(tf_pub          src/tf_pub.cpp)
# ament_target_dependencies(tf_pub
#   rclcpp tf2_ros tf2_eigen tf2_eigen_kdl
# )

# Install targets
install(
  TARGETS
    change_header
    # change_frame
    change_odom
    msg_to_tf
    # publish_tf
    # publish_horizon
    # publish_rotate
    # tf_pub
  DESTINATION lib/${PROJECT_NAME}
)

# Finalize
ament_package()