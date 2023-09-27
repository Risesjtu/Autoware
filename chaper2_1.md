# Chapter2_1 - Autoware建图模块

> 高精地图与经典建图算法概述
>
> map_file模块介绍及源码解析
>
> ndt_mapping模块介绍及源码解析
>
> 实践：基于Ndt_mapping算法建立点云地图
>
> 实践：Autoware Tools 与 Vector Map的绘制（语义信息的标注）



## 一、高精地图与经典建图算法概述

### 什么是高精地图（HD map）？

- 高精地图是一个数据体
- 数据精度为分米甚至厘米级，包含**空间信息**、**语义信息**和**时间信息**的数据体
- 空间信息--点云地图
- **语义信息--车道线、交通灯、停止线、转向路标、速度标识、人行横道、路牙**
- 时间信息（语义信息的升级版）--红绿灯信息、早晚可变车道信息、路灯白天不开晚上开



### 高精地图有哪些格式？

**Vector map**（autoware框架的主要选择）、lanelet2、opendrive、Nds

**不同格式间的高精地图可以相互转化**



### 点云地图是如何创建的？

激光雷达输出点云数据。输入点云数据一般基于激光雷达坐标系，激光雷达坐标系和车身坐标系为刚性连接。如果将车辆起始位置当成地图坐标系的原点，那么在之后的运动过程中的某些间隔均匀的时刻，如果能够**准确的获取车辆的位姿变化**，就能将原本基于激光雷达坐标系的点云信息转换到地图坐标系下，进而就构成了点云地图的一部分。以此往复，就会构成一张庞大的点云地图

某些激光SLAM算法中的做法：如果是关键帧，则把关键帧的原始点云数据直接插入到地图中，核心还是坐标变化



### 如何能够准确的获取车辆的位姿变化？

+ RTK差分定位（成本较高，需要搭建基站，不能够有遮挡例如隧道也不行）
+ **激光slam匹配算法**（存在累计误差，长时间使用会造成漂移）
+ 轮速计定位（存在累计误差，长时间使用会造成漂移）

**实际使用时，根据不同场景混用**



### 用什么工具可以在点云地图上标注语义信息？

**Autoware tools（重点介绍和使用演示）**、Unity插件版、VTD、RoadRunner（前两款免费）



### 高精地图的生产流程

**数据融合  ->  点云地图建立  ->  降采样去噪  ->  语义信息标注**

+ 数据融合：一般为相机和激光雷达形成的RGB点云数据，便于车道信息的提取（有些高级别激光雷达如128线，可能直接清晰的显示车道信息）。也有一些通过边建图边语义识别路况信息的系统，目的是为了降低后面手工语义信息标注的工作量
+ 点云建图：一般通过SLAM算法
+ 去噪：如动态障碍物也会被建图，不过这个也可以通过语义信息标注来标识



### 经典激光slam算法概述

视觉slam（VIO）：orbslam、vins、svo、dso

激光slam（2d）：gmapping、hector、Karto、cartographer2d

激光slam（3d）：loam系、cartographer3d、**Ndt**

**本课程重点关注建立3d点云地图的激光slam算法，特别是Ndt（Autoware的选择）**





**建图的关键在于位姿变化的准确估计，对于slam算法而言，位姿变化的计算是通过点云特征匹配优化后得到的**

### 根据特征匹配形式的分类

+ **Scan to Scan**：loam系（当前帧与上一帧去匹配）

  Loam会将输入scan中的点云根据**曲率**大小分为**平面点**和**边缘点**，之后的匹配优化过程也是针对当前输入scan和上衣scan的平面点和边缘点来研究进行的。根据**边缘点的距离优化公式**和**平面点的距离优化公式**来构造优化方程求解位姿变化量

  + 点云数据都是一组组（x,y,z），可以通过计算每个点和它附近点的关系，来判断每个点是属于平面点和边缘点，然后给它标识上

  Lego-loam、lio-sam等都是基于这一原理来进行位姿优化求解的、只不过他们引入了更多传感器并加入了回环检测。

+ **Scan to Map**：cartographer、Ndt（当前帧和已经建好的地图进行匹配）

  两者都是通过当前scan同已经建好的map（或者submap）来进行特征匹配的，和loam提取有曲率特征的点云不同，cartographer将当前scan通过hit（是否打在墙上）的方式来和上一次建好的submap来进行匹配优化；而**Ndt则是将map网格化后计算每个网格的均值方差，并通过当前scan中的每个点落在map网格中的正态分布概率来进行匹配优化的**（Ndt算法的具体原理在后续讲，这里先当作一个库去调用）



## 二、map_file模块介绍及源码解析

各类模块其实就相当于一个ROS package

map_file模块主要负责**读取pcd点云文件和csv语义地图文件**，解析并将其**转换成描述地图信息的topic**发送给rviz和其他功能模块

![map_file](pictures\map_file.png)

+ map_file这个ROS包包含了两个节点，也相当于两个可执行文件。（对应则有两个.cpp源码文件可以解读）

+ 这个模块的输入是pcd或者csv，输出是point_map和vector_map等话题，供rviz可视化的订阅以及其他模块如定位、感知等模块，可以订阅地图的话题

+ 重点关注两个源码文件中的关键function：

  ![map_file_function](pictures\map_file_function.png)



### 源码解读

首先打开官方demo中的new_map.launch，这个启动文件中启动的两个节点，即是map_file模块中的两个功能！

具体路径`autoware/documentation/autoware_quickstart_examples/launch/sil_env_demo/my_map.launch`

``` 
 <!-- Point Cloud -->
  <node pkg="map_file" type="points_map_loader" name="points_map_loader" args="noupdate $(env HOME)/.autoware/data/map/sil_pointcloud_map/simcity9.pcd"/>

  <!-- Vector Map -->
  <node pkg="map_file" type="vector_map_loader" name="vector_map_loader" args="$(env HOME)/.autoware/data/map/sil_vector_map/lane.csv 
```

在这个建图的启动文件中，分别启动了`points_map_loader`和`vector_map_lodaer`，在终端运行这个launch，并打开rviz可视化，呈现出的是一副建好的地图。

+ 通过左侧插件中的map文件夹可知，这幅地图包含了分别由`points_map_loader`加载的点云地图（Points Map），和`vector_map_lodaer`加载的高精地图（ADAS Map）

+ 这两幅地图的源头（原始数据）在`$(env HOME)/.autoware/data/map/sil_pointcloud_map/xxx.pcd`和`$(env HOME)/.autoware/data/map/vector_map/xxx.csv`

  通过launch文件中的`args="noupdate $(env HOME)/.autoware/data/map/sil_pointcloud_map/simcity9.pcd"`即可知道数据源头

+ 在rviz中左侧栏的插件，可以知道`Points Map`和`ADAS Map`分别订阅的是`/points_map`和`/vector_map`两个话题



#### 1. points_map_loader.cpp

路径：`autoware/common/map_file/nodes/points_map_loader/points_map_loader.cpp`

+ 由于我们在上面的`new_map.launch中`已经知道了`points_map_loader发布`的是`/points_map`这个话题，所以我们可以在.cpp文件中直接搜索`points_map"`关键词，来迅速捕捉到：这个模块的重点流程，包括话题在何处发布的，何处订阅的，节点的句柄在哪里等等。（**阅读源码不需要逐行读，我们更多的关注主体功能通过怎样一个流程来实现的，例如`points_map`这个话题的具体走向，调用了哪个函数**）

通过在vscode中`ctrl H`快速搜索，查到`points_map`话题是通过`pcd_pub`这个句柄发布的，在搜索`pcd_pub`，可以发现具体是`void publish_pcd(...)`函数发布的，继续搜索这个函数，可以最终发现：在主函数中的某处，调用了`publish_pcd`这个函数，真正实现了`points_map`话题的发布：

+ 具体代码段：

  ``` c++
  int main(int argc, char **argv)
  {
  ...
      std::string area(argv[1]);
  	if (area == "noupdate")
  		margin = -1;
  ...
      if (margin < 0) {
  	    int err = 0;
  	    publish_pcd(create_pcd(pcd_paths, &err), &err);
  ```

  虽然主函数中有多个if分支，但是对于我们这个仿真项目，只需要关注`margin<0`这个分支，因为在主函数中定义了`area`参数，这个参数用来解析launch文件中的第一个参数：在`new_map.launch`文件中，在`points_map_loader`节点启动时，通过args传入了`noupdate`参数。

  （其他if分支多与实车调试时有关，例如实车测试时，利用gnss对地图进行一个调整，暂不具体学习）



#### 2. vector_map_loader.cpp

路径：`autoware/common/map_file/nodes/vector_map_loader/vector_map_loader.cpp`

源头是`.autoware/data/map/sil_vector_map/xxx.csv文件`，在rviz中的终点是`ADAS Map`高精地图订阅的`/vector_map`话题，这个过程经历了一系列数据流的处理：

+ 在.cpp源文件的936行开始，注册了非常多的publish的句柄，每个句柄负责发布的话题都关联一个csv文件（对应某一类语义信息，如车道线、路灯等等），**这些句柄为  其他模块或者rviz 来对这些不同话题的订阅  服务！**

  或者可以说，**完成csv文件的解析后将各种矢量信息发布出去 **

+ 例如：

  ``` 
  ros::Publisher cross_walk_pub = nh.advertise<CrossWalkArray>("vector_map_info/cross_walk", 1, true);
  ```

  后续则可以通过订阅`/cross_walk`话题，来获取cross walk的相关元素信息

+ 以`crosswalk.csv`数据的解析为例：同样可以通过在.cpp文件中搜索`cross_walk_pub`，来知晓如何从原始的csv信息转换成我们自定义的`CrossWalk.msg`，再合成一个话题，发布出去！

  + 从1073行开始，通过调用之前定义的模板函数（`createObjectArray()`），来实现了每一份`.csv`文件的解析（即从csv数据 转换成 CrossWalk.msg）。即**读取"crosswalk.csv"文件中数据,创建一个`ObjectArray`，并且通过cross_walk_pub发布 **

  + 从1235行开始，**是利用`MarkerArray`这个数组**（专门用来提供可视化的数组）**将之前通过`cross_walk_pub`等pub发布过的语义信息，集合到一起，形成一个`vector_map`话题，发布出去**（相当于将语义信息二次发布了，只不过这次发布是为了rviz可视化，而打包形成了一个`vector_map`话题），这样才能供rviz等去订阅，真正实现可视化（之前`createObjectArray()`解析出来的信息并不能真正使用）

    ``` c++
    //vector_map话题的发布
      ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("vector_map", 1, true);
      
      ...
          
    //note-tianyu 定义一个"VectorMap"类型的vmap,通过subscribe来填充vmap中的各种矢量元素
      VectorMap vmap;
      vmap.subscribe(nh, category);
    //note-tianyu 定义好一个marker_array后，依次将vmap中的各种矢量元素插入到marker_array里面去，用于rviz中的vector map显示
      visualization_msgs::MarkerArray marker_array;
      insertMarkerArray(marker_array, createRoadEdgeMarkerArray(vmap, Color::GRAY));
      ...
      insertMarkerArray(marker_array, createCrossWalkMarkerArray(vmap, Color::WHITE));
      ...
    // 注释位置—把插入完各种vector_map的marker_array发布出去
      marker_array_pub.publish(marker_array);
    ```

    + 其中，`VectorMap vmap;`和`vmap.subscribe(nh, category);`，是通过`subscribe()`函数来对`vmap`进行填充，跳转阅读`subscribe()`的定义，又可以追溯到`registerSubscriber(nh, category)`函数，它实现的即是： **将.cpp的936行开始发布的一系列语义信息，又重新订阅回来，用来填充vmap集合**

  

## 三、ndt_mapping模块介绍及源码解析

ndt_mapping模块主要负责基于**传感器输入**的点云、imu、里程计等信息，**基于ndt匹配优化算法**，**建立点云地图**

![Ndt_mapping](pictures\Ndt_mapping.png)

+ `lidar_localize`模块非常重要，包括下一章要讲解的Ndt_matching、Ndt_CPU等功能都在其中
+ `lidar_localize`模块的输入是 点云等信息，输出是点云地图

**图中的虚线框代表可选输入topic，意思是除了原始点云数据必须作为该模块的输入，还可以选择加入imu或者里程计作为输入**，本项目重点介绍除了输入原始点云，额外加入imu的建图功能流程。

+ 重点关注源码文件中的接收点云topic后的**回调函数**和接收imu topic后的回调函数，以及回调函数中涉及到的几个重点`calc`函数：

![Ndt_mapping_function](pictures\Ndt_mapping_function.png)



### 源码解读

#### ndt_mapping.cpp

路径：`autoware/core_perception/lidar_localizer/nodes/ndt_mapping/ndt_mapping.cpp`

通过.cpp中1046行开始的  几行  订阅话题的代码，可以看出这个模块的输入：（原始点云、imu）

``` c++
  ros::Subscriber points_sub = nh.subscribe("points_raw", 100000, points_callback);  //回调函数
  ros::Subscriber odom_sub = nh.subscribe("/vehicle/odom", 100000, odom_callback); 
  ros::Subscriber imu_sub = nh.subscribe(_imu_topic, 100000, imu_callback);
```

同理，通过发布了什么，可以得到模块下这个功能的输出：（主要是点云地图`/ndt_map`，以及定位信息pose）

``` c++
  ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
  current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);
```



**订阅话题时，除了传入订阅的话题，还有就是订阅话题后要执行的一系列操作即回调函数**，如上面代码段中的标识：本小节的重点就是**解读两个回调函数**：`points_callback`和`imu_callback`

##### 1. points_callback()

跳转到这个回调函数的定义：（源码文件的454行  或者  选中points_callback()，右键跳转到定义）

+ 首先定义了一帧点云的`input`（初始是ROSMsg格式）

  接着调用`pcl::fromROSMsg(*input, tmp)`将其转换成pcl格式，这样有助于调用pcl的一些库进行一些点云的专业操作

  操作包括 点云的过滤（坐标和反射率的处理，以及只挑选一定范围内的点云），过滤后的点云插入到`scan`容器

  接着，将scan生成一个智能指针：`pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan)); `

+ 接下来，如果这次传入的这帧点云是第一帧点云，那就直接插入到点云地图中，即**第一帧scan输入，会将其当成直接插入到map中，作为组成map的第一帧子图 **

  ``` c++
   if (initial_scan_loaded == 0)
   {
     pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol)
     map += *transformed_scan_ptr;
     initial_scan_loaded = 1;
   }
  ```

+ + 初始的时候，车子基本相对于地图没有什么变化，即map坐标系此时就相当于车身坐标系`base_link`，所以第一帧点云插入地图的操作：只需要完成一个base_link（车身坐标系） 到 雷达坐标系的坐标变换

    + `tf_btol `变量代表这个坐标变换的tf tree，即变换矩阵。**`tf_btol `这个矩阵在源码文件的1036行，已经预设好了**，这个矩阵的信息是通过yaml文件读取出来的。**这个矩阵以及它的逆矩阵（雷达->base link）非常重要，会多次使用到**

    完成坐标变换后，通过`map += *transformed_scan_ptr;`进行点云的简单叠加，即相当于点云插入地图的操作，生成点云地图`map`

+ 对这帧点云进行体素滤波。之后，将刚刚生成的点云地图 生成一个地图智能指针，根据对ndt优化的相关参数进行配置。

  **根据ndt算法进行配置有四种方式：直接调用pcl库，单独用CPU，用GPU（如果装了CUDA），用pcl_OPENMP **

  ``` c++
  if (_method_type == MethodType::PCL_GENERIC) //调用pcl库
    {
      ndt.setTransformationEpsilon(trans_eps);
      ndt.setStepSize(step_size);
      ndt.setResolution(ndt_res);
      ndt.setMaximumIterations(max_iter);
      ndt.setInputSource(filtered_scan_ptr);
    }
    else if (_method_type == MethodType::PCL_ANH) //cpu ndt
    {...}
  #ifdef CUDA_FOUND
    else if (_method_type == MethodType::PCL_ANH_GPU) //gpu ndt
    {...}
  #endif
  #ifdef USE_PCL_OPENMP
    else if (_method_type == MethodType::PCL_OPENMP) //pcl_OPENMP
    {...}
  ```

  大部分情况直接调用pcl库，但直接阅读pcl库不是这个项目的重点，所以后续会解析autoware项目中的`cpu ndt`和`gpu ndt`的源码，尤其是CPU的ndt算法流程

  下一章的课中会重点讲解ndt算法，重点涉及一些：迭代次数，优化的步长，残差取多少等等，**主要实现了针对刚刚过滤好的点云scan和生成的点云地图map，找到scan在map中的位置**

+ **对于第一帧点云来说，后续几乎没什么操作**

+ **如果不是第一帧点云**，之前的几步操作中，就不会有直接插入地图去生成地图，因为地图已经存在，然后调用pcl库或者直接CPU等几种方式下根据对ndt优化的相关参数进行配置，都差不多

  直接跳转到560行的`guess_pose`（先验位姿的计算部分）

+ 调用`imu_odom_calc()`、`imu_calc()`、`odom_calc()`几个函数，对imu或者里程计的信息进行一个计算，将计算结果经过一些处理，然后当成**下一次ndt计算的先验信息**，ndt算法也需要一些先验的初值。（567行开始）

  演示的demo中只使用到了imu，所以我们重点解读一下`imu_clac()`

  + **跳转到`imu_clac()`的定义**：（297行）

    + 先说明一下imu的使用特性：使用的周期越长，imu的累计误差会越大，越会漂移。但如果时间很短，imu的精度比较高。**在位姿更新的时候，短时间内更相信imu的变化信息，来作为下一帧进行ndt算法时更可靠的先验**
    + `imu_clac()`函数的作用，概括来说：**在上一帧精确ndt定位位姿的基础上，再加上一个Δt内imu的精准变化，来作为下一帧ndt计算的初始值**

    通过角速度*时间，计算imu位姿变化的roll、pitch、yaw，来更新全局地图坐标系的位姿`current_pose_imu `

    计算imu在的线加速度在地图坐标系下xyz方向的线加速度分量：accX 、accY 、accZ （由于要更新的是地图坐标系的pose，所以这里的线加速度分量作了变换处理：通过左乘 “imu坐标系->地图坐标系” 的坐标变换）

    然后，利用上一步得到的加速度，通过高中公式`s = v0*t + (a*t^2)/2 `，计算得到xyz三个坐标轴的位置坐标（`offset_imu_x`、`offset_imu_y` 、`offset_imu_z`），同理也可以得到三个方向的速度

    **同理，利用`offset`变量去进行更新：例如，imu的变化位姿`offset_imu_x `加上  上一帧的位姿`previous_pose.x `，得到`guess_pose_imu.x `，为更新下一帧点云的ndt计算的先验做准备**

    + 更新的位姿信息包括了x、y、z、roll、pitch、yaw

+ 根据`imu_clac()`等几个函数的计算结果，例如`guess_pose_imu`，来更新ndt的先验位姿：

  ``` c++
    //根据imu和odo等传感器的配置情况来定义guess_pose_for_ndt
    pose guess_pose_for_ndt;
    if (_use_imu == true && _use_odom == true)
      guess_pose_for_ndt = guess_pose_imu_odom;
    else if (_use_imu == true && _use_odom == false)
      guess_pose_for_ndt = guess_pose_imu; // guess_pose_imu -> guess_pose_for_ndt
    else if (_use_imu == false && _use_odom == true)
      guess_pose_for_ndt = guess_pose_odom;
    else
      guess_pose_for_ndt = guess_pose;
  ```

+ 再经过一点变换，得到`init_guess`，作为ndt优化的初始值预测 

+ 将`init_guess`和之前得到的scan转换成地图坐标系下，两者共同带入，正式进入到ndt算法运算（后续细讲）

  ``` c++
   if (_method_type == MethodType::PCL_GENERIC) // 由此进入正式的ndt算法运算
    {...}
    else if (_method_type == MethodType::PCL_ANH)
    {...}
  ```

+ 由于定位计算的车身base link 在map坐标系下的坐标，所以要将雷达坐标系下的scan转换到map坐标系下

  - 涉及到的`t_localizer`，是雷达坐标系在map坐标系下的坐标变换

  建图的核心步骤：**根据ndt匹配优化结果将scan的坐标由lidar坐标系换算到map坐标系下 **（一直往地图坐标系投影，然后将投影的点云作为子图插入到map中）

+ 对`current_pose_imu`等current变量进行一个更新，防止出现很大的偏移

+ 计算两frame之间的距离间隔，当大于一定阈值时，才像向地图中插入新的点云（子地图） 

  **即将关键帧插入到map中**

+ 最后将map信息转换成ROSMsg格式，并发布出去：`pcl::toROSMsg(*map_ptr, *map_msg_ptr);`



##### 2. imu_callback()

imu的频率很高，差不多达到100hz，所以一直有msg进来，也就是说`imu_callback()`的调用频率很高

而imu的回调函数中，用到的最重要的函数依旧是在前面解析过的`imu_clac()`函数，

跳转到`imu_callback()`的定义：（401行）

+ imu是否需要调整方向，有时候可能涉及到imu正装倒装的问题 
+ 将四元数转成欧拉角，限制欧拉角大小不超过M_PI （imu的原始信息是四元数，所以先要转换成欧拉角）
+ **根据欧拉角变换来更新角速度**, 不相信imu直接输入的角速度值 





## map_file模块的试用

参考官方demo中关于载入pcd和csv的启动文件：`my_map.launch`

（`autoware.ai/src/autoware/documentation/autoware_quickstart_examples/launch/rosbag_demo/my_map.launch`）



利用chapter2_2中讲师提供的simcity.bag的数据集，使用`ndt_mapping`建图模块（`my_mapping.launch`）建图，并保存了simcity9.pcd格式的点云地图；

利用chapter2_3中总结的Autoware Tools标注工具，对simcity9.pcd进行了语义标注，包括路灯、信号灯、人行横道等等，并生成ADAS Map，保存为一系列.csv文件（如stopline.csv、utilitypole.csv）;



编写自己使用map_file模块的启动launch：`try_map_file.launch`  （也放在官方demo的启动launch路径下）

![try_map_file_launch](pictures\try_map_file_launch.png)

+ 其中，分别启动`points_map_loader`和`vector_map_loader`节点，分别用于载入pcd和csv的数据

+ pcd和csv文件的来源即上述simcity地图相关，并且放在autoware的文件加载目录`.autoware`下，即`.autoware/data/map/sil_pointclod_map`和`.autoware/data/map/sil_vector_map`路径下

+ **map_file模块的成功使用，还需要非常重要的一环：tf坐标变换的编写，即`world to map`、`base_link_to_velodyne`（车身到激光雷达）等一系列坐标轴的坐标变化**

  统一写入`sil_tf.launch`中，同样放在文件加载目录`.autoware`下，专门存放tf文件的路径：`data/tf`

  **没有正确写入各坐标系的tf关系，就算加载进去pcd和csv，在rviz中也显示不出任何东西**

+ 把原本官方demo中`my_localization.launch`中的，关于引入小车模型的节点，也写到自己的launch里了；

  为了方便修改和编译，把原官方demo的`my_localization.launch`中include小车相关的launch，也重新编写成`try_map_file_vehicle.launch`，但是不放在原引入vehicle相关目录下（visualization/launch），而是放在官方demo的launch同目录下

+ 为了方便，把rviz节点也直接放进来了，并且rviz配置文件使用官方demo中的`default.rviz`



### pcd和csv加载的地图效果如预期一样

![try_map_file](pictures\try_map_file.png)



成功试用了map_file模块！



### 额外说明

讲师在gitee仓库的tools分支里，上传了pcd文件保存格式的转换工具，提到了按照`bag_to_pcd`方式直接保存的pcd地图，可能map_file读取不进去，要利用工具先转换成ASCII码格式保存的pcd（和Autoware Tools一样）

**但经过我的尝试，至少在我的环境下，不论是原格式的pcd或者ASCII格式的pcd，rviz中都能清晰显示，并无区别，我猜测可能讲师的意思是，会影响autoware后续模块（如ndt_matching等）对地图的处理**

**所以，我建议养成习惯，直接保存完pcd地图后，统一先转换成ASCII格式的pcd文件**





ndt_mapping模块的试用在chapter2_2中有展示，这里不作赘述



至于官方demo中`my_localiztion.launch`的模仿，还需要下一章的知识学完，因为定位的功能除了用到了`ndt_mapping`模块，还另有如`gnss`和`ndt_matching`等后续将学习的模块
