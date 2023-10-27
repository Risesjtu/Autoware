#!/bin/bash

cd "$( dirname "${BASH_SOURCE[0]}" )"

# 仿真环境
cd ../../autoware-1.14/
gnome-terminal -t "env" -x bash -c "source install/setup.bash && roslaunch autoware_quickstart_examples simulation_env.launch;exec bash;"
# -t代表标题；-x是在一个主终端打开多个子终端，18.04推荐用--替代-x，但是--会每一条gmoe-terminal打开一个分离终端；exec bash代表在终端内容执行完也不关闭终端
sleep 10s

# 地图
gnome-terminal -t "map" -x bash -c "source install/setup.bash && roslaunch autoware_quickstart_examples simulation_map.launch;exec bash;"
sleep 2s

# 定位
gnome-terminal -t "localization" -x bash -c "source install/setup.bash && roslaunch autoware_quickstart_examples simulation_localization.launch;exec bash;"
sleep 2s