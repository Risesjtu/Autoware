# NDT 实车定位

# 加载点云地图

```bash
$ roslaunch autoware_quickstart_examples sjtu_map.launch
```

其中 `sjtu_map.launch` 文件内容如下，实际使用中需要相应地修改 `pcd` 点云地图文件列表 和 `csv` 矢量地图文件列表.

```xml
<launch>

  <!-- TF -->
  <include file="$(env HOME)/.autoware/sjtu_data/tf/tf.launch"/>

  <!-- Point Cloud -->
  <include file="$(find map_file)/launch/points_map_loader.launch">
      <arg name="path_pcd" 
           value=
           '$(env HOME)/.autoware/sjtu_data/map/pointcloud_map/autoware-220621.pcd,
            $(env HOME)/.autoware/sjtu_data/map/pointcloud_map/autoware-220628-0.pcd,
            $(env HOME)/.autoware/sjtu_data/map/pointcloud_map/autoware-220628-1.pcd,
            $(env HOME)/.autoware/sjtu_data/map/pointcloud_map/0.30_autoware-221020.pcd,
            $(env HOME)/.autoware/sjtu_data/map/pointcloud_map/0.30_autoware-221028.pcd,
            $(env HOME)/.autoware/sjtu_data/map/pointcloud_map/autoware-220929.pcd'/>
  </include>

  <!-- Vector Map -->
  <node pkg="map_file" type="vector_map_loader" name="vector_map_loader" 
    args="
        $(env HOME)/.autoware/sjtu_data/map/vector_map/dtlane.csv
        $(env HOME)/.autoware/sjtu_data/map/vector_map/lane.csv
        $(env HOME)/.autoware/sjtu_data/map/vector_map/line.csv
        $(env HOME)/.autoware/sjtu_data/map/vector_map/node.csv
        $(env HOME)/.autoware/sjtu_data/map/vector_map/point.csv
        $(env HOME)/.autoware/sjtu_data/map/vector_map/roadedge.csv
        $(env HOME)/.autoware/sjtu_data/map/vector_map/whiteline.csv"/>
  
  <node pkg="rviz" type="rviz" name="rviz" 
    args="-d $(find autoware_quickstart_examples)/launch/sjtu_demo/default.rviz"/>
  <node pkg="rqt_tf_tree" type="rqt_tf_tree" name="rqt_tf_tree" />

</launch>

```



## Debug issue#2398

- OS: Ubuntu 18.04
- ROS: Melodic
- Autoware: 1.14
- Build: Source

在运行 [官方 Demo](https://github.com/autowarefoundation/autoware_ai_documentation/wiki/ROSBAG-Demo) 时发现无法成功定位, 在 tf_tree 中, map 和 base_link 无法连接起来. 详细情况可参考 https://github.com/autowarefoundation/autoware/issues/2398#issue-1099171325.

经过检查发现, 在 `Runtime Manager / Quick Start / Map` 中启动 `my_map.launch` 后, 点云地图没有被正确发布, 在话题 `/points_map` 上没有消息.   

```
$ rostopic echo /points_map
WARNING: no messages received and simulated time is active.
Is /clock being published?
```

这是因为在 Autoware 1.14 中修改了加载点云地图的方式，却没有更改 `my_map.launch` 文件，导致点云地图加载失败。可通过如下之一的方法解决：
- 通过 `Runtime Manager` 加载点云地图
  通过 `Runtime Manager / Map / Point Cloud` 按钮导入点云地图, 待进度条加载到 100% 并显示 OK 后, 点云地图才算被正确加载, 此时运行 官方 Demo 才能定位成功.  
  通过这种方式有时也不能成功加载点云地图, 有时进度条走到一半就失败了, 重试即可.
- 修改 `my_map.launch` 文件，按如下方式加载：
  ```xml
  <!-- Point Cloud -->
  <include file="$(find map_file)/launch/points_map_loader.launch">
      <arg name="path_pcd" 
           value=
           '$(env HOME)/.autoware/sjtu_data/map/pointcloud_map/autoware-220621.pcd,
            $(env HOME)/.autoware/sjtu_data/map/pointcloud_map/autoware-220628-0.pcd,
            $(env HOME)/.autoware/sjtu_data/map/pointcloud_map/autoware-220628-1.pcd,
            $(env HOME)/.autoware/sjtu_data/map/pointcloud_map/0.30_autoware-221020.pcd,
            $(env HOME)/.autoware/sjtu_data/map/pointcloud_map/0.30_autoware-221028.pcd,
            $(env HOME)/.autoware/sjtu_data/map/pointcloud_map/autoware-220929.pcd'/>
  </include>

  ```

# 定位
```bash
$ roslaunch autoware_quickstart_examples sjtu_localization.launch
```

其中 `sjtu_localization.launch` 文件内容如下，其中主要包括如下工作:
- 设置激光雷达的装配位置 `base_link_to_localizer`
- 启动雷达点云降采样模块 `points_downsample.launch`(定位中需要将降采样后的点云`/filtered_points`与点云地图`/points_map`进行配准)
- 启动 GNSS 模块 `nmea2tfpose.launch`, 将串口读取到的 `nmea_sentence` 经纬度信息转换为相对于 126 号参考点的 `gnss_pose` 位姿, 该126号参考点的经纬度在 `geo_pos_conv.cpp` 文件中指定
- 启动 `ndt_matching.launch` 开始定位，并通过参数 `use_gnss` 表明通过 gnss_pose 给出 ndt 配准的初始化位姿，并且在 ndt 配准误差过大时，重新给定初始值进行纠正。

```xml
<launch>
  <!-- setting path parameter -->
  <arg name="get_height" value="true" />
  <!-- Setup -->
  <node pkg="tf2_ros"  type="static_transform_publisher" name="base_link_to_localizer" args="0.7 0 1.85 0 0 0 /base_link /velodyne" />
  <include file="$(find vehicle_description)/launch/vehicle_model.launch" />
  <!-- points downsampler -->
  <include file="$(find points_downsampler)/launch/points_downsample.launch" />
  <!-- points transformer -->
  <include file="$(find points_preprocessor)/launch/cloud_transformer.launch" />
  <!-- nmea2tfpose -->
  <include file="$(find gnss_localizer)/launch/nmea2tfpose.launch">
    <arg name="plane" default="126"/>
  </include>


  <param name="tf_x" value="0.7" />
  <param name="tf_y" value="0.0" />
  <param name="tf_z" value="1.85" />
  <param name="tf_yaw" value="0.0" />
  <param name="tf_pitch" value="0.0" />
  <param name="tf_roll" value="0.0" />
  <param name="localizer" value="velodyne" />
  <!-- ndt_matching -->
  <include file="$(find lidar_localizer)/launch/ndt_matching.launch">
    <arg name="get_height" value="$(arg get_height)"/> 
    <arg name="use_gnss" default="1"/>
  </include>
</launch>

```

