# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "sawyer_hardware_interface;gazebo_ros_control;image_transport;intera_core_msgs;realtime_tools;roscpp;kdl_parser;tf_conversions;sns_ik_lib".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lsawyer_robot_hw_sim".split(';') if "-lsawyer_robot_hw_sim" != "" else []
PROJECT_NAME = "sawyer_gazebo"
PROJECT_SPACE_DIR = "/home/staff/peng/ros_ws/install"
PROJECT_VERSION = "5.3.0"
