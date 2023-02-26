cmake_minimum_required(VERSION 2.8.3)

project(yutu_description)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
