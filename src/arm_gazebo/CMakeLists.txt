cmake_minimum_required(VERSION 3.10)
project(arm_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
