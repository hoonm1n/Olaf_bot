cmake_minimum_required(VERSION 2.8.3)

project(hospital_bot)

find_package(catkin REQUIRED
	geometry_msgs
  rospy
  rviz
  sensor_msgs
  std_msgs
  tf
  urdf)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(send_goal src/send_robot_goal.cpp)

target_link_libraries(send_goal
    ${catkin_LIBRARIES}
)


find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
