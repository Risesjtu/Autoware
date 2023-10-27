#!/bin/bash

# 杀死map相关进程
# ps -ef | grep sjtu_map.launch 是查询进程信息
# cut -c 9-15 是截取输入行的第9个字符到第15个字符，而这正好是进程号PID
# xargs kill -s 9 中的xargs命令是用来把前面命令的输出结果（PID）作为“kill -s 9”命令的参数，并执行该命令。“kill -s 9”会强行杀掉指定进程。
gnome-terminal -t "clear1" -- bash -c "ps -ef | grep sjtu_map.launch | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear1" -- bash -c "ps -ef | grep world_to_map | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear1" -- bash -c "ps -ef | grep map_to_mobility | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear1" -- bash -c "ps -ef | grep points_map_loader | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear1" -- bash -c "ps -ef | grep vector_map_loader | cut -c 9-15 | xargs kill -s 9 ; exec bash"
# gnome-terminal -t "clear1" -- bash -c "ps -ef | grep rqt_tf_tree | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear1" -- bash -c "ps -ef | grep rviz | cut -c 9-15 | xargs kill -s 9 ; exec bash"


# 杀死localization相关进程
gnome-terminal -t "clear2" -- bash -c "ps -ef | grep sjtu_localization.launch | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear2" -- bash -c "ps -ef | grep base_link_to_localizer | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear2" -- bash -c "ps -ef | grep velodyne_up_to_front | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear2" -- bash -c "ps -ef | grep joint_state_publisher | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear2" -- bash -c "ps -ef | grep robot_state_publisher | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear2" -- bash -c "ps -ef | grep voxel_grid_filter | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear2" -- bash -c "ps -ef | grep cloud_transformer | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear2" -- bash -c "ps -ef | grep nmea2tfpose | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear2" -- bash -c "ps -ef | grep points_concat_filter | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear2" -- bash -c "ps -ef | grep ndt_matching | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear2" -- bash -c "ps -ef | grep pub_initial_pose | cut -c 9-15 | xargs kill -s 9 ; exec bash"


# 杀死detection相关进程
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep sjtu_detection_part1.launch | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep sjtu_detection_part2.launch | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep baselink_to_ground | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep lidar_euclidean_cluster_detect | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep range_vision_fusion_01 | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep wayarea2grid | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep cluster_detect_visualization_01 | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep can_status_translator | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep pose_relay | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep vel_relay | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep calibration_publisher | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep vision_darknet_detect | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep yolo7_rects | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep ray_ground_filter | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep imm_ukf_pda_01 | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep ukf_track_relay_01 | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep ukf_track_visualization_01 | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep object_roi_filter_clustering | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep kf_filter_visualization | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear3" -- bash -c "ps -ef | grep rm_custom_area | cut -c 9-15 | xargs kill -s 9 ; exec bash"


# 杀死planning相关进程
gnome-terminal -t "clear4" -- bash -c "ps -ef | grep sjtu_mission_planning.launch | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear4" -- bash -c "ps -ef | grep op_global_planner | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear4" -- bash -c "ps -ef | grep astar_navi | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear4" -- bash -c "ps -ef | grep costmap_generator | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear4" -- bash -c "ps -ef | grep lane_rule | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear4" -- bash -c "ps -ef | grep lane_select | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear4" -- bash -c "ps -ef | grep pathPreSolve | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear4" -- bash -c "ps -ef | grep waypoint_marker_publisher | cut -c 9-15 | xargs kill -s 9 ; exec bash"


gnome-terminal -t "clear5" -- bash -c "ps -ef | grep sjtu_motion_planning.launch | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear5" -- bash -c "ps -ef | grep op_common_params | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear5" -- bash -c "ps -ef | grep op_trajectory_generator | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear5" -- bash -c "ps -ef | grep op_motion_predictor | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear5" -- bash -c "ps -ef | grep op_trajectory_evaluator | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear5" -- bash -c "ps -ef | grep op_behavior_selector | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear5" -- bash -c "ps -ef | grep astar_avoid | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear5" -- bash -c "ps -ef | grep velocity_set | cut -c 9-15 | xargs kill -s 9 ; exec bash"


gnome-terminal -t "clear7" -- bash -c "ps -ef | grep sjtu_scenario_manager.launch | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear7" -- bash -c "ps -ef | grep scenario_manager | cut -c 9-15 | xargs kill -s 9 ; exec bash"



# 杀死control相关进程
# gnome-terminal -t "clear6" -- bash -c "ps -ef | grep sjtu_control_pure_pursuit.launch | cut -c 9-15 | xargs kill -s 9 ; exec bash"
# gnome-terminal -t "clear6" -- bash -c "ps -ef | grep pure_pursuit | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear6" -- bash -c "ps -ef | grep sjtu_control_mpc.launch | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear6" -- bash -c "ps -ef | grep twist_filter | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear6" -- bash -c "ps -ef | grep twist_gate | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear6" -- bash -c "ps -ef | grep mpc_evaluate | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear6" -- bash -c "ps -ef | grep mpc_follower | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear6" -- bash -c "ps -ef | grep mpc_waypoints_converter | cut -c 9-15 | xargs kill -s 9 ; exec bash"
gnome-terminal -t "clear6" -- bash -c "ps -ef | grep caninfo | cut -c 9-15 | xargs kill -s 9 ; exec bash"

# 杀死ar识别进程
gnome-terminal -t "clear8" -- bash -c "ps -ef | grep ar_track_alvar | cut -c 9-15 | xargs kill -s 9 ; exec bash"

