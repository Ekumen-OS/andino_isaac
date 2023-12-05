cmake_minimum_required(VERSION 3.8)
project(andino_isaac)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    tools
    isaac_worlds
    andino_isaac_description
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
