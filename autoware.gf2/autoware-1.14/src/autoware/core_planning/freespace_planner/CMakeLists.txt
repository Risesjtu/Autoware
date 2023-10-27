cmake_minimum_required(VERSION 2.8.12)
project(freespace_planner)

set(USE_VISUALIZATION OFF)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(autoware_build_flags REQUIRED)
find_package(
  catkin REQUIRED COMPONENTS
    astar_search
    autoware_msgs
    geometry_msgs
    autoware_config_msgs
    libwaypoint_follower
    nav_msgs
    roscpp
    tf
    visualization_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(common_src
  src/astar_navi/astar_navi.cpp
  src/astar_navi/astar_navi_node.cpp
  src/astar_navi/path_interpolation.cpp
)


if(USE_VISUALIZATION)
  
  find_package(Threads REQUIRED)
  find_package(Protobuf CONFIG REQUIRED)
  message(STATUS "Using protobuf ${Protobuf_VERSION}")
  find_package(gRPC CONFIG REQUIRED)

  set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
  set(_REFLECTION gRPC::grpc++_reflection)
  set(_GRPC_GRPCPP gRPC::grpc++)
  message(STATUS "Using gRPC ${gRPC_VERSION}")

  include_directories(
    visualization
  )
  
  add_executable(
    astar_navi
    ${common_src}
    ${hw_proto_srcs}
    ${hw_grpc_srcs}
    visualization/visualization.grpc.pb.cc
    visualization/visualization.grpc.pb.h
    visualization/visualization.pb.cc
    visualization/visualization.pb.h
    visualization/visualization_sender.h
    visualization/visualization_sender.cc
  )

  target_link_libraries(astar_navi
    ${catkin_LIBRARIES}
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF}
  )
  target_compile_definitions(astar_navi PRIVATE VISUALIZATION)
else()
  add_executable(
    astar_navi
    ${common_src}
  )

  target_link_libraries(
    astar_navi
    ${catkin_LIBRARIES}
  )

endif()


add_dependencies(
  astar_navi
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS astar_navi
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
