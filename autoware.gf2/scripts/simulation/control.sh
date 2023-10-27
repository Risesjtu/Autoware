#!/bin/bash

cd "$( dirname "${BASH_SOURCE[0]}" )"

# mpc 
cd ../../autoware-1.14/
gnome-terminal -t "mpc" -x bash -c "source install/setup.bash && roslaunch mpc_follower mpc_follower.launch;exec bash;"
# -t代表标题；-x是在一个主终端打开多个子终端，18.04推荐用--替代-x，但是--会每一条gmoe-terminal打开一个分离终端；exec bash代表在终端内容执行完也不关闭终端
sleep 2s

# twist_filter
gnome-terminal -t "openPlanner" -x bash -c "source install/setup.bash && roslaunch twist_filter twist_filter.launch ;exec bash;"
sleep 2s

