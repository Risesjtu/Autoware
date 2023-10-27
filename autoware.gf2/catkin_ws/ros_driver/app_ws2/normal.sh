#!/bin/bash


# 地图模块
gnome-terminal -t "map" -x bash -c "source /home/kang/autoware.gf/autoware-1.14/install/setup.bash && roslaunch autoware_quickstart_examples sjtu_map.launch;exec bash;"
sleep 3s
# 定位模块
gnome-terminal -t "localization" -x bash -c "source /home/kang/autoware.gf/autoware-1.14/install/setup.bash && roslaunch autoware_quickstart_examples sjtu_localization.launch;exec bash;"
sleep 3s
# 感知模块
gnome-terminal -t "detection" -x bash -c "source /home/kang/autoware.gf/autoware-1.14/install/setup.bash && roslaunch autoware_quickstart_examples sjtu_detection.launch;exec bash;"
sleep 3s
# 规划模块

# 运动控制模块

# bag
gnome-terminal -t "rosbag" -x bash -c "rosbag play /home/kang/autoware.gf/autoware-1.14/bag_2022-07-20-22-38-43_0.bag  ;exec bash;"
