cmake_minimum_required(VERSION 3.5)
project(pe_ars408_ros)

if (NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif ()


if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()

set(autoware_perception_msgs_DIR ~/autoware.gf2/catkin_ws/ros_driver/catkin_ws_msg/devel/share/autoware_perception_msgs/cmake/)
set(can_msgs_DIR ~/autoware.gf2/catkin_ws/ros_driver/catkin_ws_control/devel/share/can_msgs/cmake/)
set(socketcan_bridge_DIR ~/autoware.gf2/catkin_ws/ros_driver/catkin_ws_control/devel/share/socketcan_bridge/cmake/)
set(socketcan_interface_DIR ~/autoware.gf2/catkin_ws/ros_driver/catkin_ws_control/devel/share/socketcan_interface/cmake/)

find_package(catkin
   REQUIRED COMPONENTS
   autoware_perception_msgs
   sensor_msgs
   can_msgs
   socketcan_bridge
   unique_id
   diagnostic_updater
   diagnostic_msgs
)

catkin_package(
   CATKIN_DEPENDS
   sensor_msgs
   can_msgs
)

include_directories(
   ${catkin_INCLUDE_DIRS}
   include
)

# Driver Library
add_library(pe_ars408_parser
   src/ars408_driver.cpp
)

# ROS node
add_executable(pe_ars408_node
   src/ars408_ros_node.cpp
)

target_link_libraries(pe_ars408_node
   pe_ars408_parser
   ${catkin_LIBRARIES}
)
# Status
add_executable(pe_ars408_status
   src/ars408_radar_status.cpp
   )

target_link_libraries(pe_ars408_status
   pe_ars408_parser
   ${catkin_LIBRARIES}
   )

####
install(TARGETS pe_ars408_node pe_ars408_parser pe_ars408_status
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(TARGETS
  pe_ars408_node pe_ars408_status
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
