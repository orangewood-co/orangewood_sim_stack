cmake_minimum_required(VERSION 3.0.2)
project(owl_description)

find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher_gui
  robot_state_publisher
  urdf
)


catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)