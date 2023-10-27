#!/bin/bash

cd "$( dirname "${BASH_SOURCE[0]}" )"

# 雷达驱动
cd ../../catkin_ws/ros_driver/catkin_ws_sensor/
gnome-terminal -t "lidar" -x bash -c "source devel/setup.bash && roslaunch rslidar_sdk start.launch;exec bash;"

# 相机驱动
gnome-terminal -t "camera" -x bash -c "source devel/setup.bash && roslaunch usb_cam usb_cam-test.launch;exec bash;"

# 超声波
gnome-terminal -t "camera" -x bash -c "source devel/setup.bash && roslaunch mc_radar mc_radar.launch ;exec bash;"

# 组合导航驱动
cd ../catkin_ws_fixposition/
gnome-terminal -t "gnss" -x bash -c "source install/setup.bash && roslaunch fixposition_driver_ros1 tcp.launch;exec bash;"
                          
# 底盘驱动
cd ../../../../ros_driver/catkin_ws_control/
gnome-terminal -t "chassis1" -x bash -c "source devel/setup.bash && roslaunch pix_driver pix_driver.launch;exec bash;"

gnome-terminal -t "chassis2" -x bash -c "source devel/setup.bash && roslaunch socketcan_bridge socketcan_bridge.launch;exec bash;"
