# Install script for directory: /home/divjot/catkin_ws/src/fetch_gazebo/fetchit_challenge

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/divjot/catkin_ws/install")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge/action" TYPE FILE FILES "/home/divjot/catkin_ws/src/fetch_gazebo/fetchit_challenge/action/SchunkMachine.action")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge/msg" TYPE FILE FILES
    "/home/divjot/catkin_ws/devel/share/fetchit_challenge/msg/SchunkMachineAction.msg"
    "/home/divjot/catkin_ws/devel/share/fetchit_challenge/msg/SchunkMachineActionGoal.msg"
    "/home/divjot/catkin_ws/devel/share/fetchit_challenge/msg/SchunkMachineActionResult.msg"
    "/home/divjot/catkin_ws/devel/share/fetchit_challenge/msg/SchunkMachineActionFeedback.msg"
    "/home/divjot/catkin_ws/devel/share/fetchit_challenge/msg/SchunkMachineGoal.msg"
    "/home/divjot/catkin_ws/devel/share/fetchit_challenge/msg/SchunkMachineResult.msg"
    "/home/divjot/catkin_ws/devel/share/fetchit_challenge/msg/SchunkMachineFeedback.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge/action" TYPE FILE FILES "/home/divjot/catkin_ws/src/fetch_gazebo/fetchit_challenge/action/SickCamera.action")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge/msg" TYPE FILE FILES
    "/home/divjot/catkin_ws/devel/share/fetchit_challenge/msg/SickCameraAction.msg"
    "/home/divjot/catkin_ws/devel/share/fetchit_challenge/msg/SickCameraActionGoal.msg"
    "/home/divjot/catkin_ws/devel/share/fetchit_challenge/msg/SickCameraActionResult.msg"
    "/home/divjot/catkin_ws/devel/share/fetchit_challenge/msg/SickCameraActionFeedback.msg"
    "/home/divjot/catkin_ws/devel/share/fetchit_challenge/msg/SickCameraGoal.msg"
    "/home/divjot/catkin_ws/devel/share/fetchit_challenge/msg/SickCameraResult.msg"
    "/home/divjot/catkin_ws/devel/share/fetchit_challenge/msg/SickCameraFeedback.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge/cmake" TYPE FILE FILES "/home/divjot/catkin_ws/build/fetch_gazebo/fetchit_challenge/catkin_generated/installspace/fetchit_challenge-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/divjot/catkin_ws/devel/include/fetchit_challenge")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/divjot/catkin_ws/devel/share/roseus/ros/fetchit_challenge")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/divjot/catkin_ws/devel/share/common-lisp/ros/fetchit_challenge")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/divjot/catkin_ws/devel/share/gennodejs/ros/fetchit_challenge")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/divjot/catkin_ws/devel/lib/python2.7/dist-packages/fetchit_challenge")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/divjot/catkin_ws/devel/lib/python2.7/dist-packages/fetchit_challenge")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/divjot/catkin_ws/build/fetch_gazebo/fetchit_challenge/catkin_generated/installspace/fetchit_challenge.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge/cmake" TYPE FILE FILES "/home/divjot/catkin_ws/build/fetch_gazebo/fetchit_challenge/catkin_generated/installspace/fetchit_challenge-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge/cmake" TYPE FILE FILES
    "/home/divjot/catkin_ws/build/fetch_gazebo/fetchit_challenge/catkin_generated/installspace/fetchit_challengeConfig.cmake"
    "/home/divjot/catkin_ws/build/fetch_gazebo/fetchit_challenge/catkin_generated/installspace/fetchit_challengeConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge" TYPE FILE FILES "/home/divjot/catkin_ws/src/fetch_gazebo/fetchit_challenge/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/fetchit_challenge" TYPE PROGRAM FILES
    "/home/divjot/catkin_ws/src/fetch_gazebo/fetchit_challenge/scripts/spawn_assembly_delayed.sh"
    "/home/divjot/catkin_ws/src/fetch_gazebo/fetchit_challenge/scripts/spawn_assembly_delayed_simple.sh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/fetchit_challenge" TYPE DIRECTORY FILES
    "/home/divjot/catkin_ws/src/fetch_gazebo/fetchit_challenge/config"
    "/home/divjot/catkin_ws/src/fetch_gazebo/fetchit_challenge/launch"
    "/home/divjot/catkin_ws/src/fetch_gazebo/fetchit_challenge/models"
    "/home/divjot/catkin_ws/src/fetch_gazebo/fetchit_challenge/urdf"
    "/home/divjot/catkin_ws/src/fetch_gazebo/fetchit_challenge/worlds"
    )
endif()

