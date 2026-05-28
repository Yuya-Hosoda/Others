cmake_minimum_required(VERSION 3.8)
project(farm_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds models config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/spawn_weeds.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
