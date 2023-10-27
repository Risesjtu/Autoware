cmake_minimum_required(VERSION 3.0.2)
project(rm_custom_area)

find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  vector_map
  autoware_msgs
  autoware_can_msgs
  nav_msgs
  geometry_msgs
  op_planner
  op_ros_helpers
  tf
  pcl_conversions
  pcl_ros
  # ompl
)
find_package(Eigen3 REQUIRED)

# generate_messages(
#   DEPENDENCIES
#   std_msgs 
#   autoware_msgs  
# )

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rm_custom_area
  CATKIN_DEPENDS 
  roscpp
  rospy
  std_msgs
  vector_map
  autoware_msgs
  autoware_can_msgs
  nav_msgs
  geometry_msgs
  op_planner
  op_ros_helpers
  tf
  pcl_conversions
  pcl_ros
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  rm_custom_area
  src/rm_custom_area.cpp
  lib/rm_custom_area_core.cpp
)

target_link_libraries(
  rm_custom_area
  ${catkin_LIBRARIES}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS
  rm_custom_area
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

