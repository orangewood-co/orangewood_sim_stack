cmake_minimum_required(VERSION 3.0.2)
project(owl_perception)


find_package(catkin REQUIRED COMPONENTS
  
  realsense_gazebo_plugin
)




catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)
