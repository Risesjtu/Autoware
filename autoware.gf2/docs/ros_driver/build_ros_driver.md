- [ros\_driver 项目结构](#ros_driver-项目结构)
- [编译catkin\_ws\_msg](#编译catkin_ws_msg)
- [编译catkin\_ws\_control](#编译catkin_ws_control)
- [编译catkin\_ws\_sensor](#编译catkin_ws_sensor)
  - [添加cmake文件路径](#添加cmake文件路径)
- [编译catkin\_ws\_camera](#编译catkin_ws_camera)
- [编译catkin\_ws\_fixposition](#编译catkin_ws_fixposition)

# ros_driver 项目结构

该部分代码位于 `autoware.gf2/catkin_ws/ros_driver` 文件夹内.

ros_driver 中的部分驱动包依赖于 autoware.ai, 因此需要先成功编译 autoware.ai, 才能执行如下的驱动编译.

```bash
$ sudo apt install tree
$ tree ros_driver -L 3
.
├── catkin_ws_msg
│   └── src
│       ├── autoware_control_msgs
│       ├── autoware_perception_msgs
│       ├── autoware_planning_msgs
│       ├── autoware_vehicle_msgs
│       ├── CMakeLists.txt
│       └── README.md
├── catkin_ws_control
│   ├── other
│   │   ├── pix_driver_msgs-master.zip
│   │   ├── pix_driver-robobus.zip
│   │   ├── points_preprocessor
│   │   ├── raw_vehicle_cmd_converter-master.zip
│   │   └── ros_canopen-pixmoving.zip
│   └── src
│       ├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
│       ├── pix_driver_msgs-master
│       ├── pix_driver-robobus
│       ├── raw_vehicle_cmd_converter-master
│       ├── ray_filter_ground
│       └── ros_canopen-pixmoving
├── catkin_ws_sensor
│   └── src
│       ├── ars408_ros-master
│       ├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
│       ├── mc_radar
│       ├── open_planner.rviz
│       └── rslidar_sdk
├── catkin_ws_camera
│   └── src
│       ├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
│       └── zed-ros-wrapper
├── catkin_ws_fixposition
│   └── src
│       ├── fixposition_driver
│       └── fixposition_gnss_tf
└── usb_cam-test.launch

```

# 编译catkin_ws_msg
如果存在其他设备的编译缓存, 直接`catkin_make`编译会报错，因为之前在其他设备上编译的缓存文件与本地路径不匹配，需要将之前的编译缓存删去.
然后就可以编译通过.   
```bash
$ cd catkin_ws/ros_driver/catkin_ws_msg/
$ rm -rf build/ devel/
$ catkin_make
...
Scanning dependencies of target autoware_planning_msgs_generate_messages
[100%] Built target autoware_perception_msgs_generate_messages_py
[100%] Built target autoware_planning_msgs_generate_messages
Scanning dependencies of target autoware_perception_msgs_generate_messages
[100%] Built target autoware_perception_msgs_generate_messages
[100%] Built target autoware_vehicle_msgs_generate_messages_py
[100%] Built target autoware_vehicle_msgs_generate_messages_eus
Scanning dependencies of target autoware_vehicle_msgs_generate_messages
[100%] Built target autoware_vehicle_msgs_generate_messages

```

# 编译catkin_ws_control
1. 删除缓存
```bash
$ cd catkin_ws/ros_driver/catkin_ws_control/
$ rm -rf build/ devel/
```
2. 编译

```bash
$ catkin_make
...
-- Could NOT find autoware_msgs (missing: autoware_msgs_DIR)
-- Could not find the required component 'autoware_msgs'. The following CMake error indicates that you either need to install the package with the same name or change your environment so that it can be found.
CMake Error at /opt/ros/melodic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "autoware_msgs"
  with any of the following names:

    autoware_msgsConfig.cmake
    autoware_msgs-config.cmake

  Add the installation prefix of "autoware_msgs" to CMAKE_PREFIX_PATH or set
  "autoware_msgs_DIR" to a directory containing one of the above files.  If
  "autoware_msgs" provides a separate development package or SDK, be sure it
  has been installed.
Call Stack (most recent call first):
  pix_driver-robobus/CMakeLists.txt:10 (find_package)

```

出现如上报错信息，仔细阅读以上内容，可知在`pix_driver-robobus/CMakeLists.txt`中查找该包，但失败了，报错信息中也给出了解决方案，就是`set "autoware_msgs_DIR"`. 因此首先要手动找到`autoware_msgsConfig.cmake`文件的位置
```bash
$ sudo find / | grep autoware_msgsConfig.cmake
/home/lsy/autoware.gf2/autoware-1.14/build/autoware_msgs/devel/share/autoware_msgs/cmake/autoware_msgsConfig.cmake
/home/lsy/autoware.gf2/autoware-1.14/build/autoware_msgs/catkin_generated/installspace/autoware_msgsConfig.cmake
/home/lsy/autoware.gf2/autoware-1.14/install/autoware_msgs/share/autoware_msgs/cmake/autoware_msgsConfig.cmake
```
优先选最后一个路径`*/install/*`,在`src/pix_driver-robobus/CMakeLists.txt`文件中`find_package(...)`之前设置路径即可，得到如下所示:
```cmake
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(autoware_msgs_DIR ~/autoware.gf2/autoware-1.14/install/autoware_msgs/share/autoware_msgs/cmake/)
find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  can_msgs
  roscpp
  std_msgs
  pix_driver_msgs
)

```

3. 按照上述步骤，需要修改的包括如下内容：   

- src/pix_driver-robobus/CMakeLists.txt
  - set(autoware_msgs_DIR ~/autoware.gf2/autoware-1.14/install/autoware_msgs/share/autoware_msgs/cmake/)
- src/raw_vehicle_cmd_converter-master/CMakeLists.txt
  - set(autoware_vehicle_msgs_DIR ~/autoware.gf2/catkin_ws/ros_driver/catkin_ws_msg/devel/share/autoware_vehicle_msgs/cmake/)
  - set(autoware_control_msgs_DIR ~/autoware.gf2/catkin_ws/ros_driver/catkin_ws_msg/devel/share/autoware_control_msgs/cmake/)
  - set(autoware_msgs_DIR ~/autoware.gf2/autoware-1.14/install/autoware_msgs/share/autoware_msgs/cmake/)

然后就能编译通过

```bash
$ catkin_make
...
[ 98%] Linking CXX executable /home/lsy/autoware.gf2/catkin_ws/ros_driver/catkin_ws_control/devel/lib/canopen_chain_node/canopen_chain_node
[ 98%] Built target canopen_chain_node
[ 98%] Linking CXX shared library /home/lsy/autoware.gf2/catkin_ws/ros_driver/catkin_ws_control/devel/lib/libcanopen_motor.so
[ 98%] Built target canopen_motor
Scanning dependencies of target canopen_motor_node
[100%] Building CXX object ros_canopen-pixmoving/canopen_motor_node/CMakeFiles/canopen_motor_node.dir/src/canopen_motor_chain_node.cpp.o
[100%] Linking CXX executable /home/lsy/autoware.gf2/catkin_ws/ros_driver/catkin_ws_control/devel/lib/canopen_motor_node/canopen_motor_node
[100%] Built target canopen_motor_node

```

4. 补充
   如果编译过程报错: `muparser library not found`, 直接安装即可.
   ```bash
   $  sudo apt install libmuparser-dev
   ```

# 编译catkin_ws_sensor
## 添加cmake文件路径
```bash
$ cd catkin_ws/ros_driver/catkin_ws_sensor
$ rm -rf build/ devel/
```

- src/ars408_ros-master/CMakeLists.txt
  - set(autoware_perception_msgs_DIR ~/autoware.gf2/catkin_ws/ros_driver/catkin_ws_msg/devel/share/autoware_perception_msgs/cmake/)
  - set(can_msgs_DIR ~/autoware.gf2/catkin_ws/ros_driver/catkin_ws_control/devel/share/can_msgs/cmake/)
  - set(socketcan_bridge_DIR ~/autoware.gf2/catkin_ws/ros_driver/catkin_ws_control/devel/share/socketcan_bridge/cmake/)
  - set(socketcan_interface_DIR ~/autoware.gf2/catkin_ws/ros_driver/catkin_ws_control/devel/share/socketcan_interface/cmake/)

```bash
$ cd ~/autoware.gf2/catkin_ws/ros_driver/catkin_ws_sensor && catkin_make
$ source devel/setup.bash && rospack profile
```



# 编译catkin_ws_camera
0. 直接编译会报错，缺少ZED SDK

```bash
$ cd catkin_ws_camera/
$ catkin_make
...
-- ==> add_subdirectory(zed-ros-wrapper/zed_nodelets)
CMake Warning at zed-ros-wrapper/zed_nodelets/CMakeLists.txt:22 (find_package):
  By not providing "FindZED.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "ZED", but
  CMake did not find one.

  Could not find a package configuration file provided by "ZED" (requested
  version 3) with any of the following names:

    ZEDConfig.cmake
    zed-config.cmake

  Add the installation prefix of "ZED" to CMAKE_PREFIX_PATH or set "ZED_DIR"
  to a directory containing one of the above files.  If "ZED" provides a
  separate development package or SDK, be sure it has been installed.


CMake Error at zed-ros-wrapper/zed_nodelets/CMakeLists.txt:17 (message):
  

  

   ZED SDK v3.x not found, install it from:
   https://www.stereolabs.com/developers/ 

  

Call Stack (most recent call first):
  zed-ros-wrapper/zed_nodelets/CMakeLists.txt:23 (checkPackage)



```
	 
1. 从官网下载 ZED SDK：[ZED SDK 3.6 - Download | Stereolabs](https://www.stereolabs.com/developers/release/3.6/), 该网站需要科学上网.
2. 需安装 CUDA, 教程：[CUDA Toolkit 12.0 Update 1 Downloads | NVIDIA Developer](https://developer.nvidia.com/cuda-downloads)
	 
3. SDK 安装说明：[How to Install ZED SDK on Linux | Stereolabs](https://www.stereolabs.com/docs/installation/linux/) 
	 
4. ZED 官方例程：[stereolabs/zed-examples: ZED SDK Example projects (github.com)](https://github.com/stereolabs/zed-examples)
	 
5. 编译 ZED 相机驱动  
   本仓库 `catkin_ws/ros_driver/catkin_ws_camera/src` 路径下 有相机驱动功能包 `zed-ros-wrapper`, 该包源码托管在 [Github 仓库](https://github.com/stereolabs/zed-ros-wrapper).
	具体编译教程参考 [zed-ros-wrapper/README.md](../../catkin_ws/ros_driver/catkin_ws_camera/src/zed-ros-wrapper/README.md)


# 编译catkin_ws_fixposition
这部分是组合导航 [Fixposition Vision-RTK 2](https://www.fixposition.com/product) 的驱动程序
- [官方 Github 账号](https://github.com/fixposition)
- https://github.com/fixposition/fixposition_gnss_tf
- https://github.com/fixposition/fixposition_driver

1. 下载驱动代码(本仓库下已包含)

```bash
$ cd ros_driver
$ mkdir -p catkin_ws_fixposition/src
$ cd catkin_ws_fixposition/src
$ git clone https://github.com/fixposition/fixposition_gnss_tf.git
$ git clone https://github.com/fixposition/fixposition_driver.git
$ cd ../..
```

```
$ tree catkin_ws_fixposition/ -L 3
catkin_ws_fixposition/
└── src
    ├── fixposition_driver
    │   ├── fixposition_driver_lib
    │   ├── fixposition_driver_ros1
    │   ├── fixposition_driver_ros2
    │   ├── fixposition_odometry_converter
    │   ├── LICENSE
    │   ├── README.md
    │   └── test
    └── fixposition_gnss_tf
        ├── cmake
        ├── CMakeLists.txt
        ├── Doxyfile
        ├── include
        ├── LICENSE
        ├── package.xml
        ├── README.md
        ├── src
        └── test

12 directories, 7 files
```

2. 安装依赖库

```bash
$ sudo apt update
$ sudo apt install -y wget
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt install -y build-essential cmake
$ sudo apt install -y libeigen3-dev
$ sudo apt install -y libyaml-cpp-dev
```

3. 安装依赖库 gtest

如果不安装, 直接编译, 可能会由于找不到`GTest`而显示如下报错信息:  

```bash
ros_driver/catkin_ws_fixposition$ catkin build fixposition_gnss_tf

CMake Error at /usr/share/cmake-3.10/Modules/FindPackageHandleStandardArgs.cmake:137 (message):
  Could NOT find GTest (missing: GTEST_LIBRARY GTEST_MAIN_LIBRARY)
Call Stack (most recent call first):
  /usr/share/cmake-3.10/Modules/FindPackageHandleStandardArgs.cmake:378 (_FPHSA_FAILURE_MESSAGE)
  /usr/share/cmake-3.10/Modules/FindGTest.cmake:196 (FIND_PACKAGE_HANDLE_STANDARD_ARGS)
  cmake/testing.cmake:9 (find_package)
  CMakeLists.txt:71 (include)
```

执行如下指令安装 `GTest`:  

```bash
sudo apt-get install libgtest-dev
cd /usr/src/gtest
sudo mkdir build
cd build
sudo cmake ..
sudo make
sudo cp libgtest*.a /usr/local/lib
```

- 参考 [ubuntu 安装 google Gtest | 博客园](https://www.cnblogs.com/jessica-jie/p/6704388.html)


4. 编译  

在[官方文档](https://github.com/fixposition/fixposition_gnss_tf#build)中推荐在 `ROS1` 中使用 `catkin build` 进行编译, 在 `ROS2` 中使用 `colcon build` 进行编译.   
针对本项目的 `ROS1: ROS Melodic` 环境, 使用 `catkin build` 编译时会报如下错误:  

```bash
ros_driver/catkin_ws_fixposition$ catkin build fixposition_gnss_tf

CMake Error at /home/liyuxiang/autoware.gf/catkin_ws/ros_driver/catkin_ws_fixposition/src/fixposition_gnss_tf/cmake/testing.cmake:59 (target_link_directories):
  Unknown CMake command "target_link_directories".
Call Stack (most recent call first):
  CMakeLists.txt:74 (add_gtest)
```

这可能是因为本地使用的 CMake 版本过低(例如 3.10), 不支持 `testing.cmake` 文件中的 `target_link_directories` 指令, 经过测试, 升级 CMake 可以解决该问题, 但可能同时破坏系统中的其他依赖, 需要重新安装多个 ros 功能包才能最终解决.

- https://cmake.org/cmake/help/latest/command/target_link_directories.html
- https://cmake.org/cmake/help/latest/command/link_directories.html
- [正确的方式升级ubuntu的cmake | 博客园](https://www.cnblogs.com/Maker-Liu/p/16550381.html)

实际上, 在 ROS1 中也是可以用 colcon build 进行编译的(毕竟autoware就是这么编译的), 直接通过如下指令就可以完成组合导航所有驱动的编译:  

```
$ colcon build --packages-select fixposition_gnss_tf fixposition_driver_lib fixposition_driver_ros1
```

此外还有两个包:   
- `fixposition_odometry_converter` 是负责集成轮速计输入的(该包暂时无用,可以忽略); 
- `fixposition_driver_ros2` 是 `ROS2` 环境下的驱动包, 可以删除.
