cmake_minimum_required(VERSION 3.5)
project(linorobot2_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY launch rviz meshes urdf gazebo_worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
