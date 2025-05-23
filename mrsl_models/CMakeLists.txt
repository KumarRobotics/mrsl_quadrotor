cmake_minimum_required(VERSION 3.10)
project(mrsl_models)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)
ament_package()
