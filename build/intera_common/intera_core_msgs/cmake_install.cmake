# Install script for directory: /home/staff/peng/ros_ws/src/intera_common/intera_core_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/staff/peng/ros_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/intera_core_msgs/msg" TYPE FILE FILES
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/AnalogIOState.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/AnalogIOStates.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/AnalogOutputCommand.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/RobotAssemblyState.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/CameraControl.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/CameraSettings.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/CollisionAvoidanceState.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/CollisionDetectionState.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/DigitalIOState.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/DigitalIOStates.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/DigitalOutputCommand.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/EndpointNamesArray.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/EndpointState.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/EndpointStates.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/HeadPanCommand.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/HeadState.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/HomingCommand.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/HomingState.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/InteractionControlCommand.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/InteractionControlState.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/IOComponentCommand.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/IOComponentConfiguration.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/IOComponentStatus.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/IODataStatus.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/IODeviceConfiguration.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/IODeviceStatus.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/IONodeConfiguration.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/IONodeStatus.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/IOStatus.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/JointCommand.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/JointLimits.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/NavigatorState.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/NavigatorStates.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/SEAJointState.msg"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/msg/URDFConfiguration.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/intera_core_msgs/srv" TYPE FILE FILES
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/srv/IOComponentCommandSrv.srv"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/srv/SolvePositionFK.srv"
    "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/srv/SolvePositionIK.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/intera_core_msgs/action" TYPE FILE FILES "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/action/CalibrationCommand.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/intera_core_msgs/msg" TYPE FILE FILES
    "/home/staff/peng/ros_ws/devel/share/intera_core_msgs/msg/CalibrationCommandAction.msg"
    "/home/staff/peng/ros_ws/devel/share/intera_core_msgs/msg/CalibrationCommandActionGoal.msg"
    "/home/staff/peng/ros_ws/devel/share/intera_core_msgs/msg/CalibrationCommandActionResult.msg"
    "/home/staff/peng/ros_ws/devel/share/intera_core_msgs/msg/CalibrationCommandActionFeedback.msg"
    "/home/staff/peng/ros_ws/devel/share/intera_core_msgs/msg/CalibrationCommandGoal.msg"
    "/home/staff/peng/ros_ws/devel/share/intera_core_msgs/msg/CalibrationCommandResult.msg"
    "/home/staff/peng/ros_ws/devel/share/intera_core_msgs/msg/CalibrationCommandFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/intera_core_msgs/cmake" TYPE FILE FILES "/home/staff/peng/ros_ws/build/intera_common/intera_core_msgs/catkin_generated/installspace/intera_core_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/staff/peng/ros_ws/devel/include/intera_core_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/staff/peng/ros_ws/devel/share/roseus/ros/intera_core_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/staff/peng/ros_ws/devel/share/common-lisp/ros/intera_core_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/staff/peng/ros_ws/devel/share/gennodejs/ros/intera_core_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/staff/peng/miniconda3/envs/SpotDogPy38/bin/python3" -m compileall "/home/staff/peng/ros_ws/devel/lib/python3/dist-packages/intera_core_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/staff/peng/ros_ws/devel/lib/python3/dist-packages/intera_core_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/staff/peng/ros_ws/build/intera_common/intera_core_msgs/catkin_generated/installspace/intera_core_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/intera_core_msgs/cmake" TYPE FILE FILES "/home/staff/peng/ros_ws/build/intera_common/intera_core_msgs/catkin_generated/installspace/intera_core_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/intera_core_msgs/cmake" TYPE FILE FILES
    "/home/staff/peng/ros_ws/build/intera_common/intera_core_msgs/catkin_generated/installspace/intera_core_msgsConfig.cmake"
    "/home/staff/peng/ros_ws/build/intera_common/intera_core_msgs/catkin_generated/installspace/intera_core_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/intera_core_msgs" TYPE FILE FILES "/home/staff/peng/ros_ws/src/intera_common/intera_core_msgs/package.xml")
endif()

