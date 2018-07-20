cmake_minimum_required(VERSION 2.8.3)
project(intros)
#set(massage_generation_DIR "/opt/ros/kinetic/share/message_generation/cmake/")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  #message_generation
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  #CATKIN_DEPENDS message_runtime
)
#  INCLUDE_DIRS include
#  LIBRARIES wheel_module
#  CATKIN_DEPENDS massage_generation roscpp rospy std_msgs
#  DEPENDS system_lib


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS bin/node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
