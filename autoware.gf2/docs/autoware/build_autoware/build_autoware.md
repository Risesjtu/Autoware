# Autoware.AI
- [Autoware.AI | Github 仓库](https://github.com/autowarefoundation/autoware/tree/autoware-ai)
- [Autoware.AI Wiki](https://github.com/autowarefoundation/autoware_ai_documentation/wiki) 
- [Autoware.AI Wiki>>源码编译](https://github.com/autowarefoundation/autoware_ai_documentation/wiki/Source-Build)
# 编译 autoware
- [安装OpenCV](./installOpenCV.md)
- [更新Eigen库](update_eigen.md)
- 编译(without cuda)
  ```bash
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```
- 编译(with cuda)
  ```bash
  AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```
- 编译指定包   
  ```bash
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select autoware_quickstart_examples # 仅编译autoware_quickstart_examples功能包
  ```
# 编译失败解决办法
- 编译calibration_publisher失败
  ```bash
  $ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select calibration_publisher
  --- stderr: calibration_publisher                                                                                                
  CMakeFiles/calibration_publisher.dir/src/calibration_publisher.cpp.o: In function `main':
  calibration_publisher.cpp:(.text.startup+0x9b4): undefined reference to `cv::read(cv::FileNode const&, std::__cxx11::basic_string<char, std::char_traits<char>,   std::allocator<char> >&, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&)'
  collect2: error: ld returned 1 exit status
  make[2]: *** [devel/lib/calibration_publisher/calibration_publisher] Error 1
  make[1]: *** [CMakeFiles/calibration_publisher.dir/all] Error 2
  make: *** [all] Error 2
  Failed   <<< calibration_publisher  [ Exited with code 2 ]
  ```
  - 修改`~/autoware.ai/src/autoware/utilities/calibration_publisher/`文件夹下的`CMakeLists.txt`和`package.xml`, 添加依赖库`${OpenCV_LIBS}`, `<depend>libopencv-dev</depend>`.
  - [This error occurs when the opencv library cannot be loaded.](https://answers.ros.org/question/332086/error-when-building-package-of-calibration_publisher-while-installing-autoware/?answer=364586#post-id-364586)
  - [ [fix melodic] Add OpenCV libraries to the linker #2090 ](https://github.com/Autoware-AI/autoware.ai/pull/2090/commits/63abbb1c4d26be67ea0312b04b6dd9918cef3978)

- 编译 version_beyond_track 出错
  ```
  error: conversion from 'c::Mat’ to non-scalar type 'CvMat’ reguested
  ```
  检查 opencv 版本，尝试把 opencv 版本改为'3.2.0'
  
- 编译 autoware_health_tracker、roi_object_filter失败，找不到 `xxxConfig.cmake` 文件
  安装对应的插件或库
    ```
    sudo apt-get install ros-melodic-jsk-recognition-msgs
    sudo apt-get install ros-melodic-jsk-rviz-plugins
    sudo apt-get install ros-melodic-lanelet2-core
    sudo apt-get install ros-melodic-lanelet2-io
    sudo apt-get install ros-melodic-lanelet2-maps
    sudo apt-get install ros-melodic-grid-map-cv
    sudo apt-get install ros-melodic-grid_map
    sudo apt-get install ros-melodic-nmea-msgs
    ...
    ```
- 例如提示找不到 `nmea_navsat_driverConfig.cmake` 文件, 可以通过 `apt search` 命令查找 `ros-melodic` 相关包, 并通过 `| grep` 来过滤出与 `navsat` 相关的包, 然后安装即可.
  ```
  $ apt search ros-melodic | grep navsat

  WARNING: apt does not have a stable CLI interface. Use with caution in scripts.

  ros-melodic-nmea-navsat-driver/bionic 0.5.2-1bionic.20221025.191448 amd64
  $ sudo apt install ros-melodic-nmea-navsat-driver
  ```

- 软件依赖
  - 如果编译 autoware 遇到 gazebo 相关的报错, 尝试把 gazebo 版本改为 `9.0.0`  
  也可以重新安装完整版的 ROS 来解决:   
  `$ sudo apt install ros-melodic-desktop-full`
  - 如果编译 autoware 遇到 protobuf 相关的报错, 尝试把 protobuf 版本改为 `3.0.0`;  
    大概率是因为安装 cartographer 时使用了 3.4.1 版本的 protobuf, 导致 autoware 编译出错, 实测发现 3.0.0 版本能够兼容两者。  
    参考: [安装 protobuf 3.0.0](installprotobuf3.md)
  - 如果编译 autoware 遇到 opencv 相关的报错, 尝试把 opencv 版本改为 `3.2.0`. 
   