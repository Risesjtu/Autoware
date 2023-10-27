cmake_minimum_required(VERSION 2.12.0)
project(op_global_planner)

find_package(autoware_build_flags REQUIRED)
find_package(Eigen3 QUIET)

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(
  catkin REQUIRED COMPONENTS
    autoware_can_msgs
    geometry_msgs
    autoware_msgs
    jsk_recognition_msgs
    libwaypoint_follower
    op_planner
    op_ros_helpers
    op_simu
    op_utility
    pcl_conversions
    pcl_ros
    roscpp
    std_msgs
    tf
    vector_map_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES op_global_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(
  op_global_planner
  nodes/op_global_planner.cpp
  nodes/op_global_planner_core.cpp
  nodes/rs.cpp
)

target_link_libraries(op_global_planner ${catkin_LIBRARIES})

add_dependencies(op_global_planner ${catkin_EXPORTED_TARGETS})

install(
  TARGETS op_global_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
        
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
