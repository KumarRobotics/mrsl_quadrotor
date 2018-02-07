cmake_minimum_required(VERSION 2.8.3)
project(mrsl_quadrotor_utils)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

#find_package(catkin_simple REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp tf2_ros topic_tools eigen_conversions tf_conversions)
find_package(cmake_modules)
find_package(Eigen REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
 
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp 
#  DEPENDS)

add_executable(change_header src/change_header.cpp)
target_link_libraries(change_header ${catkin_LIBRARIES} )
add_executable(change_frame src/change_frame.cpp)
target_link_libraries(change_frame ${catkin_LIBRARIES})
add_executable(change_odom src/change_odom.cpp)
target_link_libraries(change_odom ${catkin_LIBRARIES})
add_executable(msg_to_tf src/msg_to_tf.cpp)
target_link_libraries(msg_to_tf ${catkin_LIBRARIES})
add_executable(publish_tf src/publish_tf.cpp)
target_link_libraries(publish_tf ${catkin_LIBRARIES})
add_executable(publish_horizon src/publish_horizon.cpp)
target_link_libraries(publish_horizon ${catkin_LIBRARIES})
add_executable(publish_rotate src/publish_rotate.cpp)
target_link_libraries(publish_rotate ${catkin_LIBRARIES})
 
#cs_install()
 
#cs_export()
