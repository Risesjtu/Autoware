cmake_minimum_required(VERSION 3.0.2)
project(raw_vehicle_cmd_converter)
add_compile_options(-std=c++14)

set(autoware_vehicle_msgs_DIR ~/autoware.gf2/catkin_ws/ros_driver/catkin_ws_msg/devel/share/autoware_vehicle_msgs/cmake/)
set(autoware_control_msgs_DIR ~/autoware.gf2/catkin_ws/ros_driver/catkin_ws_msg/devel/share/autoware_control_msgs/cmake/)
set(autoware_msgs_DIR ~/autoware.gf2/autoware-1.14/install/autoware_msgs/share/autoware_msgs/cmake/)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  autoware_vehicle_msgs
  autoware_msgs
  geometry_msgs
  pix_driver_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES accel_map_converter
  CATKIN_DEPENDS roscpp std_msgs autoware_vehicle_msgs geometry_msgs autoware_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(accel_map_converter
  src/accel_map.cpp
  src/brake_map.cpp
  src/csv_loader.cpp
  src/interpolate.cpp
)
target_link_libraries(accel_map_converter ${catkin_LIBRARIES})
add_dependencies(accel_map_converter ${catkin_EXPORTED_TARGETS})

add_executable(raw_vehicle_cmd_converter_node
  src/node.cpp
  src/main.cpp
)
add_dependencies(raw_vehicle_cmd_converter_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(raw_vehicle_cmd_converter_node ${catkin_LIBRARIES} accel_map_converter)

add_executable(test_node
  src/test.cpp
)
add_dependencies(test_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_node ${catkin_LIBRARIES} accel_map_converter)

install(TARGETS
    accel_map_converter
    raw_vehicle_cmd_converter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    launch
    data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)