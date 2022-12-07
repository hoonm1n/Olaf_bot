################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(Olaf_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
  #usb_cam
  std_srvs
  Olaf_description
  message_generation
)
add_message_files(
  FILES
  To_odom.msg
)


add_service_files(
  FILES
  ResetOdom.srv
  request_to_motor.srv
  
)


################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

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
  CATKIN_DEPENDS rospy roscpp sensor_msgs nav_msgs geometry_msgs tf Olaf_description message_runtime #usb_cam
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
  nodes/omo_r1_motor_node.py
  src/olaf_push.py
  src/olaf_movegoal.py
  src/tu_listener.py
  src/tu_talker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(odom_hh src/odom_hh.cpp)
add_dependencies(odom_hh ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_hh ${catkin_LIBRARIES})

add_executable(pub_vel src/pub_vel.cpp)
add_dependencies(pub_vel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_vel ${catkin_LIBRARIES})

add_executable(velCal src/velCal.cpp)
add_dependencies(velCal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velCal ${catkin_LIBRARIES})

add_executable(odom_sh src/odom_sh.cpp)
add_dependencies(odom_sh ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_sh ${catkin_LIBRARIES})

add_executable(velCal_1 src/velCal_1.cpp)
add_dependencies(velCal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velCal_1 ${catkin_LIBRARIES})
################################################################################
# Test
################################################################################
