cmake_minimum_required(VERSION 3.0.2)
project(ar_location)

find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar_msgs
  geometry_msgs
  nav_msgs
  roscpp
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ar_location
#  CATKIN_DEPENDS ar_track_alvar_msgs geometry_msgs nav_msgs roscpp
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(get_ar_info src/get_ar_info.cpp)
target_link_libraries(get_ar_info ${catkin_LIBRARIES})


add_library(myPoint include/myPoint.h src/myPoint.cpp)
add_dependencies(myPoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(myPoint ${catkin_LIBRARIES})


add_library(arLocation include/arLocation.h src/arLocation.cpp)
add_dependencies(arLocation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arLocation ${catkin_LIBRARIES})


add_executable(ar_location src/ar_location.cpp src/myPoint.cpp src/arLocation.cpp)
target_link_libraries(ar_location myPoint arLocation ${catkin_LIBRARIES})
