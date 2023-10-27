cmake_minimum_required(VERSION 3.0.2)
project(app_client)

set(autoware_msgs_DIR ../../../../../autoware-1.14/install/autoware_msgs/share/autoware_msgs/cmake/)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pix_driver_msgs
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pub_sub_test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(app_client src/main.cpp)

target_link_libraries(
  app_client
  jsoncpp
  ${catkin_LIBRARIES}
)

add_dependencies(app_client ${catkin_EXPORTED_TARGETS})

