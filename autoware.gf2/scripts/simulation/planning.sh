#!/bin/bash

cd "$( dirname "${BASH_SOURCE[0]}" )"

# 场景管理
cd ../../autoware-1.14/
gnome-terminal -t "scenario" -x bash -c "source install/setup.bash && roslaunch autoware_quickstart_examples simulation_mission_planner.launch;exec bash;"
# -t代表标题；-x是在一个主终端打开多个子终端，18.04推荐用--替代-x，但是--会每一条gmoe-terminal打开一个分离终端；exec bash代表在终端内容执行完也不关闭终端
sleep 2s

# openPlanner 
gnome-terminal -t "openPlanner" -x bash -c "source install/setup.bash && roslaunch autoware_quickstart_examples sjtu_motion_planning.launch;exec bash;"
sleep 2s

# # # astar navi
gnome-terminal -t "astarNavi" -x bash -c "source install/setup.bash && roslaunch autoware_quickstart_examples simulation_costmap_generator.launch;exec bash;"
sleep 2s