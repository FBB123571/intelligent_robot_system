cmake_minimum_required(VERSION 3.0.2)
project(show_trajectory)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES show_trajectory
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(show_trajectory src/show_trajectory.cpp)
target_link_libraries(show_trajectory ${catkin_LIBRARIES})
