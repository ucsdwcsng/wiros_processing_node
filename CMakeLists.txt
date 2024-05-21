cmake_minimum_required(VERSION 3.0.2)
project(csi_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp ")

## optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG -ffast-math -fno-math-errno")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        rf_msgs
        message_generation
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SaveChannel.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS std_msgs sensor_msgs roscpp rospy message_runtime rf_msgs
)


#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/aoa_node.py
  scripts/generate_static_compensation.py
  scripts/bag2mat.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)