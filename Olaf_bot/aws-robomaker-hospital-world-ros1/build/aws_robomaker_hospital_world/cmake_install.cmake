# Install script for directory: /home/minji/catkin_ws/src/aws-robomaker-hospital-world-ros1

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/minji/catkin_ws/src/aws-robomaker-hospital-world-ros1/install/aws_robomaker_hospital_world")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/minji/catkin_ws/src/aws-robomaker-hospital-world-ros1/build/aws_robomaker_hospital_world/catkin_generated/installspace/aws_robomaker_hospital_world.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_hospital_world/cmake" TYPE FILE FILES
    "/home/minji/catkin_ws/src/aws-robomaker-hospital-world-ros1/build/aws_robomaker_hospital_world/catkin_generated/installspace/aws_robomaker_hospital_worldConfig.cmake"
    "/home/minji/catkin_ws/src/aws-robomaker-hospital-world-ros1/build/aws_robomaker_hospital_world/catkin_generated/installspace/aws_robomaker_hospital_worldConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_hospital_world" TYPE FILE FILES "/home/minji/catkin_ws/src/aws-robomaker-hospital-world-ros1/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_hospital_world" TYPE DIRECTORY FILES
    "/home/minji/catkin_ws/src/aws-robomaker-hospital-world-ros1/launch"
    "/home/minji/catkin_ws/src/aws-robomaker-hospital-world-ros1/models"
    "/home/minji/catkin_ws/src/aws-robomaker-hospital-world-ros1/fuel_models"
    "/home/minji/catkin_ws/src/aws-robomaker-hospital-world-ros1/worlds"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/minji/catkin_ws/src/aws-robomaker-hospital-world-ros1/build/aws_robomaker_hospital_world/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/minji/catkin_ws/src/aws-robomaker-hospital-world-ros1/build/aws_robomaker_hospital_world/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
