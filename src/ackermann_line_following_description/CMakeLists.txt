cmake_minimum_required(VERSION 3.8)
project(ackermann_line_following_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
