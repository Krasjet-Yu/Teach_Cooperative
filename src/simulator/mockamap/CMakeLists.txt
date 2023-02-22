cmake_minimum_required(VERSION 3.0.2)
project(mockamap)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES mockamap
 CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


file(GLOB ${PROJECT_NAME}_SRCS src/*.cpp)

add_executable(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_SRCS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)