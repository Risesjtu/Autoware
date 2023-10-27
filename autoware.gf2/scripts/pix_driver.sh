cd "$( dirname "${BASH_SOURCE[0]}" )"
cd ../catkin_ws/ros_driver/catkin_ws_control
source devel/setup.bash
sudo chmod +x ./src/pix_driver-robobus/src/*.py
roslaunch pix_driver pix_driver_start.launch