# Install script for directory: /home/jrv/MyStuff/Advanced Robotics/kuka_experimental_ws/src/kuka_experimental/kuka_lbr_iiwa_support

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jrv/MyStuff/Advanced Robotics/kuka_experimental_ws/install")
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

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jrv/MyStuff/Advanced Robotics/kuka_experimental_ws/build/kuka_experimental/kuka_lbr_iiwa_support/catkin_generated/installspace/kuka_lbr_iiwa_support.pc")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_lbr_iiwa_support/cmake" TYPE FILE FILES
    "/home/jrv/MyStuff/Advanced Robotics/kuka_experimental_ws/build/kuka_experimental/kuka_lbr_iiwa_support/catkin_generated/installspace/kuka_lbr_iiwa_supportConfig.cmake"
    "/home/jrv/MyStuff/Advanced Robotics/kuka_experimental_ws/build/kuka_experimental/kuka_lbr_iiwa_support/catkin_generated/installspace/kuka_lbr_iiwa_supportConfig-version.cmake"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_lbr_iiwa_support" TYPE FILE FILES "/home/jrv/MyStuff/Advanced Robotics/kuka_experimental_ws/src/kuka_experimental/kuka_lbr_iiwa_support/package.xml")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_lbr_iiwa_support" TYPE DIRECTORY FILES
    "/home/jrv/MyStuff/Advanced Robotics/kuka_experimental_ws/src/kuka_experimental/kuka_lbr_iiwa_support/config"
    "/home/jrv/MyStuff/Advanced Robotics/kuka_experimental_ws/src/kuka_experimental/kuka_lbr_iiwa_support/launch"
    "/home/jrv/MyStuff/Advanced Robotics/kuka_experimental_ws/src/kuka_experimental/kuka_lbr_iiwa_support/urdf"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_lbr_iiwa_support/meshes/lbr_iiwa_14_r820/" TYPE DIRECTORY FILES "/home/jrv/MyStuff/Advanced Robotics/kuka_experimental_ws/src/kuka_experimental/kuka_lbr_iiwa_support/meshes/lbr_iiwa_14_r820/collision")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/kuka_lbr_iiwa_support/meshes/lbr_iiwa_14_r820/" TYPE DIRECTORY FILES "/home/jrv/MyStuff/Advanced Robotics/kuka_experimental_ws/src/kuka_experimental/kuka_lbr_iiwa_support/meshes/lbr_iiwa_14_r820/visual")
endif()

