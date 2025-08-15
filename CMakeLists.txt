cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamic_object_removal
  CATKIN_DEPENDS 
    rospy
    roscpp
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_sensor_msgs
    tf2_geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark Python scripts as executable
catkin_install_python(PROGRAMS
  scripts/dynamic_point_remover.py
  scripts/pointcloud_map_builder.py
  scripts/map_utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)