cmake_minimum_required(VERSION 3.16.0)
project(sharework_cembre_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config 
    launch 
    urdf
    mesh
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()