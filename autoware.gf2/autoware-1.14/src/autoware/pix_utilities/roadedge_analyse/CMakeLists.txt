cmake_minimum_required(VERSION 3.0.2)
project(roadedge_analyse)

find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  vector_map
  autoware_msgs
  geometry_msgs
  op_planner
  visualization_msgs
  tf2_ros
  vector_map_msgs
  message_generation
  pcl_conversions
  pcl_ros
  op_ros_helpers
  tf
  # ompl
)

add_message_files(FILES LaneType.msg)

generate_messages(
  DEPENDENCIES
  std_msgs 
  autoware_msgs  
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roadedge_analyse
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    vector_map
    autoware_msgs
    geometry_msgs
    op_planner
    visualization_msgs
    tf2_ros
    vector_map_msgs
    message_runtime
    op_ros_helpers
    tf
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  roadedge_analyse
  src/roadedge_analyse.cpp
  src/rs.cpp
  lib/roadedge_analyse_core.cpp
)

target_link_libraries(
  roadedge_analyse
  ${catkin_LIBRARIES}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS
    roadedge_analyse
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
