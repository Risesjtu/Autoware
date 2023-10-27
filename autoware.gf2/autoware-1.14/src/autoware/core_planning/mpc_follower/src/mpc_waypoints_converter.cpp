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

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <autoware_msgs/Lane.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "libwaypoint_follower/libwaypoint_follower.h"
#include "math.h"
#include <geometry_msgs/Quaternion.h>
#include <amathutils_lib/amathutils.hpp>

class MPCWaypointsConverter
{
public:
  MPCWaypointsConverter()
  {
    pub_waypoints_ = nh_.advertise<autoware_msgs::Lane>("/mpc_waypoints", 1);
    pub_mpc_reverse_ = nh_.advertise<std_msgs::Bool>("/mpc_reverse", 1);
    sub_closest_waypoint_ = nh_.subscribe("/closest_waypoint", 1, &MPCWaypointsConverter::callbackClosestWaypoints, this);
    sub_base_waypoints_ = nh_.subscribe("/base_waypoints_cut", 1, &MPCWaypointsConverter::callbackBaseWaypoints, this);
    sub_final_waypoints_ = nh_.subscribe("/final_waypoints", 1, &MPCWaypointsConverter::callbackFinalWaypoints, this);
    sub_pose_ = nh_.subscribe("/current_pose", 1, &MPCWaypointsConverter::callbackPose, this);
    closest_idx_ = 0;
    back_waypoints_num_ = 10;
    front_waypoints_num_ = 50;
    is_reverse_ = false;
    idx_offset_ = 0;
  };
  ~MPCWaypointsConverter(){};

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_waypoints_, pub_mpc_reverse_;
  ros::Subscriber sub_closest_waypoint_, sub_base_waypoints_, sub_final_waypoints_, 
    sub_current_velocity_, sub_pose_;

  autoware_msgs::Lane base_waypoints_;
  geometry_msgs::PoseStamped pose_;

  int closest_idx_;
  int back_waypoints_num_;
  int front_waypoints_num_;
  int idx_offset_;

  bool is_reverse_;
  bool is_basewaypoint_init_;
  bool is_current_pose_init_;
  bool is_closest_idx_init_;

  ros::Time prev_closest_idx_time;
  ros::Time prev_current_pose_time;
  ros::Time prev_basewaypoints_time;

  void callbackClosestWaypoints(const std_msgs::Int32 msg)
  {
    closest_idx_ = msg.data;
    is_closest_idx_init_ = true;
    prev_closest_idx_time = ros::Time::now();
  }

  void callbackBaseWaypoints(const autoware_msgs::Lane &msg)
  {
    base_waypoints_ = msg;
    if(!is_basewaypoint_init_){
      idx_offset_ += base_waypoints_.waypoints.size();
    }
    is_basewaypoint_init_ = true;
    prev_current_pose_time = ros::Time::now();
  }

  void callbackPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    pose_.header.stamp = msg->header.stamp;
    pose_.pose = msg->pose;
    is_current_pose_init_ = true;
    prev_basewaypoints_time = ros::Time::now();
  }

  bool checkTime(const ros::Time &t){
    if(t.toSec()-prev_closest_idx_time.toSec() > 2.0 ||
       t.toSec()-prev_current_pose_time.toSec() > 2.0 ||
       t.toSec()-prev_basewaypoints_time.toSec() > 2.0){
        return false;
       }
    return true;
  }

  // void getLocalClosestWaypoint(const autoware_msgs::Lane& waypoints, const geometry_msgs::Pose& pose, int search_size = 30)
  // {
  //   static autoware_msgs::Lane local_waypoints;  // around self-vehicle
  //   const int prev_index = closest_idx_;

  //   // search in all waypoints if lane_select judges you're not on waypoints
  //   if (closest_idx_ == -1)
  //   {
  //     // std::cout<<"pre closet_local_index: "<<closest_local_index_<<std::endl;
  //     closest_idx_ = getClosestWaypoint(waypoints, pose);
  //   }
  //   // search in limited area based on prev_index
  //   else
  //   {
  //     // get neighborhood waypoints around prev_index
  //     int start_index = std::max(0, prev_index - search_size / 2);
  //     int end_index = std::min(prev_index + search_size / 2, (int)waypoints.waypoints.size());
  //     auto start_itr = waypoints.waypoints.begin() + start_index;
  //     auto end_itr = waypoints.waypoints.begin() + end_index;
  //     local_waypoints.waypoints = std::vector<autoware_msgs::Waypoint>(start_itr, end_itr);

  //     // get closest waypoint in neighborhood waypoints
  //     int llocal_index = getClosestWaypoint(local_waypoints, pose);
  //     if(llocal_index != -1)
  //         closest_idx_ = start_index + getClosestWaypoint(local_waypoints, pose);
  //     else
  //         closest_idx_ = prev_index;
  //   }
  // }

  void callbackFinalWaypoints(const autoware_msgs::Lane &final_waypoints)
  {
    if(final_waypoints.waypoints.size() <= 2){
      ROS_WARN("The finalwaypoints too short!");
      return;
    }

    if(!is_current_pose_init_ || !is_basewaypoint_init_ || !is_closest_idx_init_){
      std::cout<<"current pose= "<<is_current_pose_init_<<std::endl;
      std::cout<<"base waypoint= "<<is_basewaypoint_init_<<std::endl;
      std::cout<<"closest idx= "<<is_closest_idx_init_<<std::endl;
      return;
    }

    // if(!checkTime(ros::Time::now())){
    //   ROS_ERROR("some msg is time out!");
    //   return;
    // }

    if (base_waypoints_.waypoints.size() == 0 || final_waypoints.waypoints.size() == 0){
      std::cout<<"base_waypoints size = "<<base_waypoints_.waypoints.size()<<std::endl;
      std::cout<<"final_waypoints size = "<<final_waypoints.waypoints.size()<<std::endl;
      return;
    }

    if ((int)base_waypoints_.waypoints.size() - 1 < closest_idx_)
    {
      closest_idx_ = getClosestWaypoint(base_waypoints_, pose_.pose);
    }

    if ((int)base_waypoints_.waypoints.size() - 1 < closest_idx_)
    {
      ROS_WARN("base_waypoints_.waypoints.size() - 1 = %d, closest_idx_ = %d", (int)base_waypoints_.waypoints.size(), closest_idx_);
      return;
    }

    auto sq_dist = [](const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
      const double dx = a.x - b.x;
      const double dy = a.y - b.y;
      return dx * dx + dy * dy;
    };

    autoware_msgs::Lane mpc_waypoints;
    mpc_waypoints.header = final_waypoints.header;
    mpc_waypoints.increment = final_waypoints.increment;
    mpc_waypoints.lane_id = final_waypoints.lane_id;
    mpc_waypoints.lane_index = final_waypoints.lane_index;
    mpc_waypoints.cost = final_waypoints.cost;
    mpc_waypoints.closest_object_distance = final_waypoints.closest_object_distance;
    mpc_waypoints.closest_object_velocity = final_waypoints.closest_object_velocity;
    mpc_waypoints.is_blocked = final_waypoints.is_blocked;

    // find closest point index in base_waypoints (topic /closest_waypoints has no consistency with /final_waypoints due to delay)
    int closest_idx = -1;
    for (int i = 0; i < (int)base_waypoints_.waypoints.size(); ++i) {
      const double d = sq_dist(final_waypoints.waypoints[1].pose.pose.position, base_waypoints_.waypoints[i].pose.pose.position);
      if (d < 0.01) {
        closest_idx = i;
        break;
      }
    }
    if (closest_idx == -1) {
      ROS_ERROR("cannot find closest base_waypoints' waypoint to final_waypoints.waypoint[1] !!");
    }

    int base_start = std::max(closest_idx - back_waypoints_num_, 0);
    for (int i = base_start; i < closest_idx; ++i)
    {
      mpc_waypoints.waypoints.push_back(base_waypoints_.waypoints.at(i));
      mpc_waypoints.waypoints.back().twist = final_waypoints.waypoints[1].twist;
    }

    int final_end = std::min(front_waypoints_num_ + 1, (int)final_waypoints.waypoints.size());
    for (int i = 1; i < final_end; ++i)
    {
      mpc_waypoints.waypoints.push_back(final_waypoints.waypoints.at(i));
    }
    IsDrivingForward(mpc_waypoints);
    // std::cout<<"is reverse = "<<is_reverse_<<std::endl;
    std_msgs::Bool reverse_msg;
    reverse_msg.data = is_reverse_;
    pub_mpc_reverse_.publish(reverse_msg);
    pub_waypoints_.publish(mpc_waypoints);
  }

  void IsDrivingForward(autoware_msgs::Lane &traj){
    if(traj.waypoints.size() < 3){
      ROS_INFO("MPC traj has not enough point!");
      return;
    }
    int count = 0;
    double current_yaw = tf2::getYaw(pose_.pose.orientation);
    double traj_yaw = tf2::getYaw(traj.waypoints.at(2).pose.pose.orientation);
    // double real_yaw = 0.0;
    for(auto &p: traj.waypoints){
      if(p.twist.twist.linear.x < 0){
        count++;
      }
    }
    double dx = traj.waypoints.at(2).pose.pose.position.x - traj.waypoints.at(0).pose.pose.position.x;
    double dy = traj.waypoints.at(2).pose.pose.position.y - traj.waypoints.at(0).pose.pose.position.y;
    double real_yaw = atan2(dy, dx);
    // std::cout<<"real = "<<real_yaw<<", traj yaw = "<<traj_yaw<<std::endl;
    
    if(count / traj.waypoints.size() > 0.8 || abs(amathutils::normalizeRadian(real_yaw - traj_yaw)) > 2.3){
      is_reverse_ = true;
    }else{
      is_reverse_ = false;
    }

    if(is_reverse_){
      for(auto &p: traj.waypoints){
        traj_yaw = traj_yaw + M_PI;
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(traj_yaw);
        p.pose.pose.orientation = q;
      }
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_waypoints_converter");
  MPCWaypointsConverter obj;
  ros::spin();
  return 0;
};
