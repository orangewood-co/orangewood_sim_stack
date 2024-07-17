cmake_minimum_required(VERSION 3.0.2)
project(owl_gazebo)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  joint_trajectory_controller
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

