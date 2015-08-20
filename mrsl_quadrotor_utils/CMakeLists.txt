cmake_minimum_required(VERSION 2.8.3)
project(mrsl_quadrotor_utils)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin_simple REQUIRED)
find_package(cmake_modules)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
 
catkin_simple()
 
cs_add_executable(change_header src/change_header.cpp)
cs_add_executable(change_frame src/change_frame.cpp)
cs_add_executable(change_odom src/change_odom.cpp)
cs_add_executable(msg_to_tf src/msg_to_tf.cpp)
#cs_add_executable(publish_tf src/publish_tf.cpp)
#cs_add_library(global_odom src/global_odom.cpp)
#cs_add_executable(transform_odom src/transform_odom.cpp)
#cs_add_executable(image_flip src/image_flip.cpp)
#cs_add_executable(play_tf_static src/play_tf_static.cpp)
 
cs_install()
 
cs_export()


