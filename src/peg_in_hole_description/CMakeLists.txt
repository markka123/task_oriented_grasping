cmake_minimum_required(VERSION 3.8)
project(peg_in_hole_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf srdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
