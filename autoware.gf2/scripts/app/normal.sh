#!/bin/bash

cd "$( dirname "${BASH_SOURCE[0]}" )"

# 地图模块
cd ../../autoware-1.14/
gnome-terminal -t "map" -x bash -c "source install/setup.bash && roslaunch autoware_quickstart_examples sjtu_map.launch;exec bash;"
# -t代表标题；-x是在一个主终端打开多个子终端，18.04推荐用--替代-x，但是--会每一条gmoe-terminal打开一个分离终端；exec bash代表在终端内容执行完也不关闭终端
sleep 3s

# 定位模块
gnome-terminal -t "localization" -x bash -c "source install/setup.bash && roslaunch autoware_quickstart_examples sjtu_localization.launch;exec bash;"
sleep 4s
# gnome-terminal -t "localization" -x bash -c "source install/setup.bash && rosrun lidar_localizer pub_initial_pose.py;exec bash;"
# sleep 3s

# 感知模块
gnome-terminal -t "detection" -x bash -c "source install/setup.bash && roslaunch autoware_quickstart_examples sjtu_detection_part1.launch;exec bash;"
sleep 5s

gnome-terminal -t "detection" -x bash -c "source install/setup.bash && roslaunch autoware_quickstart_examples sjtu_detection_part2.launch;exec bash;"
sleep 5s

# 规划模块
gnome-terminal -t "scenario_manager" -x bash -c "source install/setup.bash && roslaunch autoware_quickstart_examples sjtu_scenario_manager.launch;exec bash;"
sleep 3s
gnome-terminal -t "mission_planning" -x bash -c "source install/setup.bash && roslaunch autoware_quickstart_examples sjtu_mission_planning.launch;exec bash;"
sleep 3s
gnome-terminal -t "motion_planning" -x bash -c "source install/setup.bash && roslaunch autoware_quickstart_examples sjtu_motion_planning.launch;exec bash;"
sleep 4s

# 控制模块
gnome-terminal -t "control" -x bash -c "source install/setup.bash && roslaunch autoware_quickstart_examples sjtu_control_mpc.launch;exec bash;"
sleep 3s

# ar识别
# cd ../catkin_ws/ros_perception/ar_dock/
# gnome-terminal -t "control" -x bash -c "source devel/setup.bash && roslaunch ar_track_alvar ljc_dock.launch;exec bash;"
