cmake_minimum_required(VERSION 3.5)
project(ros2_control_demo_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY diffbot/urdf diffbot/rviz
  DESTINATION share/${PROJECT_NAME}/diffbot
)

install(
  DIRECTORY carlikebot/meshes carlikebot/urdf carlikebot/rviz
  DESTINATION share/${PROJECT_NAME}/carlikebot
)

ament_package()
