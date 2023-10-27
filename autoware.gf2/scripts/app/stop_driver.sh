#!/bin/bash
gnome-terminal -t "clear1" -x bash -c "ps -ef | grep start.launch | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear1" -x bash -c "ps -ef | grep rslidar_sdk_node | cut -c 9-15 | xargs kill -s 9 ; exec bash"

gnome-terminal -t "clear2" -x bash -c "ps -ef | grep usb_cam-test.launch | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear2" -x bash -c "ps -ef | grep usb_cam_front | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear2" -x bash -c "ps -ef | grep usb_cam_back | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear2" -x bash -c "ps -ef | grep mc_radar_node | cut -c 9-15 | xargs kill -s 9 ; exec bash"



gnome-terminal -t "clear3" -x bash -c "ps -ef | grep tcp.launch | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -x bash -c "ps -ef | grep fixposition_driver_ros1 | cut -c 9-15 | xargs kill -s 9 ; exec bash"

gnome-terminal -t "clear4" -x bash -c "ps -ef | grep pix_driver.launch | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear4" -x bash -c "ps -ef | grep report_node | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear4" -x bash -c "ps -ef | grep command_node | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear4" -x bash -c "ps -ef | grep read_converter | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear4" -x bash -c "ps -ef | grep control_converter | cut -c 9-15 | xargs kill -s 9 ; exec bash"

gnome-terminal -t "clear5" -x bash -c "ps -ef | grep socketcan_bridge.launch | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear5" -x bash -c "ps -ef | grep socketcan_bridge | cut -c 9-15 | xargs kill -s 9 ; exec bash"
