cmake_minimum_required(VERSION 3.5)
project(mrsl_quadrotor_description)

find_package(ament_cmake REQUIRED)
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)
ament_package()

