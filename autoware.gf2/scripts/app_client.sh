#!/bin/bash

cd "$( dirname "${BASH_SOURCE[0]}" )"
cd ../catkin_ws/ros_driver/app_ws
source devel/setup.bash
roslaunch app_client sjtu_app.launch
