#!/bin/bash

cd "$( dirname "${BASH_SOURCE[0]}" )"

# 发布终点
cd ../../autoware-1.14/
gnome-terminal -t "PubGoal" -x bash -c "source install/setup.bash && rosrun scenario_manager pub_goals.py;exec bash;"