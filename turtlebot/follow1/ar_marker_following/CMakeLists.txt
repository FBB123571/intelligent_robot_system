cmake_minimum_required(VERSION 3.0.2)
project(ar_marker_following)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  ar_track_alvar
  ar_track_alvar_msgs
  tf2_ros
  tf2_geometry_msgs
  competition_core
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    geometry_msgs
    sensor_msgs
    std_msgs
    ar_track_alvar
    ar_track_alvar_msgs
    tf2_ros
    tf2_geometry_msgs
    competition_core
)

catkin_install_python(PROGRAMS
  scripts/velocity_gate.py
  scripts/marker_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
