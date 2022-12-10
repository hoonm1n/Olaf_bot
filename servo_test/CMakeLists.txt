################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(servo_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  geometry_msgs
  std_srvs
  std_msgs
  message_generation
)
#add_message_files(
#  FILES
#  
#)


add_service_files(
  FILES
  request_to_motor.srv
  sig_dy.srv
)


################################################################################
# Setup for python modules and scripts
################################################################################
##catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy roscpp sensor_msgs geometry_msgs message_runtime
)


################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/mg955_test.py
  src/pub_signal.py
  src/IR.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
