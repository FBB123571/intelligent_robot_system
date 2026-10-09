cmake_minimum_required(VERSION 3.0.2)
project(path_planning)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_planning
#  CATKIN_DEPENDS actionlib move_base_msgs roscpp
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(simple_navigation_goal src/simple_navigation_goal.cpp)
target_link_libraries(simple_navigation_goal ${catkin_LIBRARIES})

