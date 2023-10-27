/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ASTAR_NAVI_H
#define ASTAR_NAVI_H

#include <iostream>
#include <vector>
#include <cmath>
#include <utility>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/ScenarioCmd.h>
#include <freespace_planner/path_interpolation.h>

#include "astar_search/astar_search.h"

#ifdef VISUALIZATION
 #include "visualization_sender.h"
#endif

using namespace PathInterpolation;

class AstarNavi
{
public:
  AstarNavi();
  ~AstarNavi();
  void run();

private:
  // ros
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher lane_pub_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber current_pose_sub_;
  ros::Subscriber goal_pose_sub_;
  ros::Subscriber sub_scenario;
  tf::TransformListener tf_listener_;

  // params
  bool activate_scenario;
  double waypoints_velocity_;   // constant velocity on planned waypoints [km/h]
  double update_rate_;          // replanning and publishing rate [Hz]
  bool use_back_;


  // classes
  AstarSearch astar_;

  // variables
  nav_msgs::OccupancyGrid costmap_;
  geometry_msgs::PoseStamped current_pose_local_, current_pose_global_;
  geometry_msgs::PoseStamped goal_pose_local_, goal_pose_global_;
  tf::Transform local2costmap_;  // local frame (e.g. velodyne) -> costmap origin

  bool costmap_initialized_;
  bool current_pose_initialized_;
  bool goal_pose_initialized_;
  bool new_goal_;  // 增设一个标志位，判断是否有新的目标点

  WaypointReplanner replanner_;

#ifdef VISUALIZATION
  Sender sender;
  visualization::Frame vehicle;
  visualization::Frame path_original;
  visualization::Frame path_resample;

  void curve_sender(Sender &sender, visualization::Frame &msg, autoware_msgs::Lane &lane);
  void polygon_sender(Sender &sender, visualization::Frame &msg, geometry_msgs::PoseStamped &current_pose);
#endif

  // functions, callback
  void costmapCallback(const nav_msgs::OccupancyGrid& msg);
  void currentPoseCallback(const geometry_msgs::PoseStamped& msg);
  void goalPoseCallback(const geometry_msgs::PoseStamped& msg);
  void callbackGetScenario(const autoware_msgs::ScenarioCmd &msg);
  void convertFrameId();

  // fucntions
  tf::Transform getTransform(const std::string& from, const std::string& to);
  // void publishWaypoints(const nav_msgs::Path& path, const double& velocity);
  void publishWaypoints(const nav_msgs::Path& path, const double& velocity, const std::vector<bool>& velocity_flag);
  void publishStopWaypoints();
  void resampleAstarPath(const autoware_msgs::Lane &original_lane, autoware_msgs::Lane &resample_lane);
  std::pair<nav_msgs::Path, std::vector<bool>> removeOutliers(const nav_msgs::Path& path_, const std::vector<bool>& velocity_flag);
  std::vector<size_t> getReversingIndices(const std::vector<bool>& velocity_flag);
};

#endif
