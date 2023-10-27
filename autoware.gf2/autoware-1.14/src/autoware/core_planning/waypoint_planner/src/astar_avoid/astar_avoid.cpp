/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include "waypoint_planner/astar_avoid/astar_avoid.h"

// 本节点 是A*的局部规划，其实就是一些状态的转移：例如RELAYING、AVOIDING、PLANNING、STOPPING等，然后在什么状态要做什么事。
// 例如如果遇到障碍物，那要先变成STOPPING状态，即车辆停下来，然后切换到PLANNING状态，去做一个A*规划

AstarAvoid::AstarAvoid()
  : nh_()
  , private_nh_("~")
  , closest_waypoint_index_(-1)
  , obstacle_waypoint_index_(-1)
  , closest_local_index_(-1)
  , costmap_initialized_(false)
  , current_pose_initialized_(false)
  , current_velocity_initialized_(false)
  , base_waypoints_initialized_(false)
  , closest_waypoint_initialized_(false)
  , terminate_thread_(false)
{
  private_nh_.param<int>("safety_waypoints_size", safety_waypoints_size_, 100);
  private_nh_.param<double>("update_rate", update_rate_, 10.0);

  private_nh_.param<bool>("enable_avoidance", enable_avoidance_, false);
  private_nh_.param<double>("avoid_waypoints_velocity", avoid_waypoints_velocity_, 10.0);
  private_nh_.param<double>("avoid_start_velocity", avoid_start_velocity_, 5.0);
  private_nh_.param<double>("replan_interval", replan_interval_, 2.0);
  private_nh_.param<int>("search_waypoints_size", search_waypoints_size_, 50);
  private_nh_.param<int>("search_waypoints_delta", search_waypoints_delta_, 2);
  private_nh_.param<int>("closest_search_size", closest_search_size_, 30);

  // /safety_waypoints是本节点输出的一条局部安全的轨迹，会送到后续的velocity_set节点中，进行一些速度规划
  safety_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("safety_waypoints", 1, true);
  costmap_sub_ = nh_.subscribe("costmap", 1, &AstarAvoid::costmapCallback, this);
  current_pose_sub_ = nh_.subscribe("current_pose", 1, &AstarAvoid::currentPoseCallback, this);
  current_velocity_sub_ = nh_.subscribe("current_velocity", 1, &AstarAvoid::currentVelocityCallback, this);
  base_waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &AstarAvoid::baseWaypointsCallback, this);  // 在launch中 remap 了这个话题名为base_waypoints_cut，为了和倒车节点配合。这里订阅pathpresolve节点分段后的子轨迹，然后astar_avoid去局部A*
  closest_waypoint_sub_ = nh_.subscribe("closest_waypoint", 1, &AstarAvoid::closestWaypointCallback, this);
  obstacle_waypoint_sub_ = nh_.subscribe("obstacle_waypoint", 1, &AstarAvoid::obstacleWaypointCallback, this);
  sub_scenario = nh_.subscribe("scenario_manager/scenario_cmd",1,&AstarAvoid::callbackGetScenario, this);  // 订阅场景管理命令来控制本节点是否生效

  rate_ = new ros::Rate(update_rate_);
  activate_scenario_ = false;
}

AstarAvoid::~AstarAvoid()
{
  publish_thread_.join();
}

// 一些对应的回调函数，costmap、current_pose...

void AstarAvoid::costmapCallback(const nav_msgs::OccupancyGrid& msg)
{
  costmap_ = msg;
  tf::poseMsgToTF(costmap_.info.origin, local2costmap_);
  costmap_initialized_ = true;
}

void AstarAvoid::currentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  current_pose_global_ = msg;

  if (!enable_avoidance_)
  {
    current_pose_initialized_ = true;
  }
  else
  {
    current_pose_local_.pose = transformPose(
        current_pose_global_.pose, getTransform(costmap_.header.frame_id, current_pose_global_.header.frame_id));
    current_pose_local_.header.frame_id = costmap_.header.frame_id;
    current_pose_local_.header.stamp = current_pose_global_.header.stamp;
    current_pose_initialized_ = true;
  }
}

void AstarAvoid::currentVelocityCallback(const geometry_msgs::TwistStamped& msg)
{
  current_velocity_ = msg;
  current_velocity_initialized_ = true;
}

void AstarAvoid::baseWaypointsCallback(const autoware_msgs::Lane& msg)
{
  static autoware_msgs::Lane prev_base_waypoints;
  if(msg.increment != 1){
    ROS_INFO("Not astar path!");
    return;
  }
  base_waypoints_ = msg;
  // global_waypoints_ = msg;


  if (base_waypoints_initialized_)
  {
    // detect waypoint change by timestamp update
    // ros::Time t1 = prev_base_waypoints.header.stamp;
    // ros::Time t2 = base_waypoints_.header.stamp;
    // std::cout<<"t1: "<<t1.toSec()<<" t2: "<<t2.toSec()<<std::endl;

    // if (fabs((t2 - t1).toSec())> 1e-3)
    // {
    //   ROS_INFO("Receive new /base_waypoints, reset waypoint index.");
    //   closest_local_index_ = -1; // reset local closest waypoint
    //   prev_base_waypoints = base_waypoints_;
    // }
 
    // linkx add
    int pre_size = prev_base_waypoints.waypoints.size();
    int cur_size = base_waypoints_.waypoints.size();
    if(pre_size != cur_size)
    {
        ROS_INFO("Receive new /base_waypoints, reset waypoint index.");
        closest_local_index_ = -1; // reset local closest waypoint
        prev_base_waypoints = base_waypoints_;
    }
  }
  else
  {
    prev_base_waypoints = base_waypoints_;
  }

  base_waypoints_initialized_ = true;

}

void AstarAvoid::closestWaypointCallback(const std_msgs::Int32& msg)
{
  closest_waypoint_index_ = msg.data;

  if (closest_waypoint_index_ == -1)
  {
    closest_local_index_ = -1; // reset local closest waypoint
  }

  closest_waypoint_initialized_ = true;
}

void AstarAvoid::obstacleWaypointCallback(const std_msgs::Int32& msg)
{
  obstacle_waypoint_index_ = msg.data;
}

// 订阅场景管理命令的回调函数，控制本节点是否生效
void AstarAvoid::callbackGetScenario(const autoware_msgs::ScenarioCmd &msg){
  if(msg.scenario == 1 || msg.scenario == 2){
    activate_scenario_ = true;
    ROS_INFO("receieve parking scenario in astar_avoid!");
  }
  else if(msg.mode == 1){
    activate_scenario_ = false;
    ROS_INFO("receive exclusive scenario order, shutdown current scenario!");
  }

  else if(msg.mode == 0)
    activate_scenario_ = true;
}

// 本节点的主函数
void AstarAvoid::run()
{
  // check topics
  state_ = AstarAvoid::STATE::INITIALIZING;

  while (ros::ok())
  {
    ros::spinOnce();
    // 如果订阅到了所有的topic，checkInitialized()为true，则跳出“等待”循环
    if (checkInitialized())  // 在checkInitialized()中，额外要判断active_scenario_是否为true
    {
      break;
    }
    ROS_WARN("Waiting for subscribing topics...");
    ros::Duration(1.0).sleep();
  }
   
  

  // main loop
  int end_of_avoid_index = -1;
  ros::WallTime start_plan_time = ros::WallTime::now();
  ros::WallTime start_avoid_time = ros::WallTime::now();

  // reset obstacle index
  obstacle_waypoint_index_ = -1;

  // relaying mode at startup
  state_ = AstarAvoid::STATE::RELAYING;

  // start publish thread
  // 额外启动一个publish线程，即在规划避障的过程中，即时还未规划出来，都要先发出一些路径点，否则小车的运动会有问题
  publish_thread_ = std::thread(&AstarAvoid::publishWaypoints, this);

  while (ros::ok())
  {
    ros::spinOnce();
    
    // relay mode
    if (!enable_avoidance_)
    {
      rate_->sleep();
      continue;
    }

    // avoidance mode
    // 如果是大于0的数，就表示在前方不远距离处有障碍物
    bool found_obstacle = (obstacle_waypoint_index_ >= 0);
    // avoid_velocity是避障速度，通常来说避障速度要比正常行驶速度慢，判断一下现在速度是否已经慢于避障速度，那么就不用有减速操作了
    bool avoid_velocity = (current_velocity_.twist.linear.x < avoid_start_velocity_ / 3.6);

    // update state
    // 避让过程中的状态机：RELAYING、STOPPING、PLANNING、AVOIDING。简单来说，遇到障碍物了，能规划就去避障，不能规划就维持停车状态
    if (state_ == AstarAvoid::STATE::RELAYING)
    {
      // RELAYING是正常行驶状态，没有检测到障碍物时，维持正常行驶
      avoid_waypoints_ = base_waypoints_;

      if (found_obstacle)
      {
        ROS_INFO("RELAYING -> STOPPING, Decelerate for stopping");
        state_ = AstarAvoid::STATE::STOPPING;
      }
    }

    else if (state_ == AstarAvoid::STATE::STOPPING)
    {
      // STOPPING是遇到障碍物了，要先停车，停车后才能进行一个规划
      bool replan = ((ros::WallTime::now() - start_plan_time).toSec() > replan_interval_);

      // 先判断了一下障碍物是否一下子消失了，如果障碍物一下没了，那也不用启动A* planning。如果障碍物一直在，那就进入到PLANNING状态
      if (!found_obstacle)
      {
        ROS_INFO("STOPPING -> RELAYING, Obstacle disappers");
        state_ = AstarAvoid::STATE::RELAYING;
      }
      else if (replan && avoid_velocity)
      {
        ROS_INFO("STOPPING -> PLANNING, Start A* planning");
        state_ = AstarAvoid::STATE::PLANNING;
      }
    }

    else if (state_ == AstarAvoid::STATE::PLANNING)
    {
      start_plan_time = ros::WallTime::now();

      // planAvoidWaypoints()是核心避障操作
      if (planAvoidWaypoints(end_of_avoid_index))  // end_of_avoid_index是避障路径的终点
      {
        ROS_INFO("PLANNING -> AVOIDING, Found path");
        state_ = AstarAvoid::STATE::AVOIDING;  // 规划成功，进入AVOIDING状态
        start_avoid_time = ros::WallTime::now();
      }
      else
      {
        ROS_INFO("PLANNING -> STOPPING, Cannot find path");
        state_ = AstarAvoid::STATE::STOPPING;  // 规划失败，就回到STOPPING状态
      }
    }

    else if (state_ == AstarAvoid::STATE::AVOIDING)
    {
      // check一下是否达到了避障的终点，如果达到了，就回到RELAYING状态
      /*
        其实避让路径就是突出去的一段弯曲路径，走完这一段凸起的避障路径，就会回到之前正常行驶的直线路径。所以AVOIDING状态是判断是否走到了弯曲路径的最后一点
      */
      bool reached = (getLocalClosestWaypoint(avoid_waypoints_, current_pose_global_.pose, closest_search_size_) > end_of_avoid_index);
      if (reached)
      {
        ROS_INFO("AVOIDING -> RELAYING, Reached goal");
        state_ = AstarAvoid::STATE::RELAYING;
      }
      else if (found_obstacle && avoid_velocity)
      {
        bool replan = ((ros::WallTime::now() - start_avoid_time).toSec() > replan_interval_);
        if (replan)
        {
          ROS_INFO("AVOIDING -> STOPPING, Abort avoiding");
          state_ = AstarAvoid::STATE::STOPPING;
        }
      }
    }
    rate_->sleep();
  }

  terminate_thread_ = true;
}

// 检查几个重要话题是否都订阅到了
bool AstarAvoid::checkInitialized()
{
  bool initialized = false;

  // check for relay mode
  initialized = (current_pose_initialized_ && closest_waypoint_initialized_ && base_waypoints_initialized_ &&
                 (closest_waypoint_index_ >= 0) && activate_scenario_);  // activate_scenario_在这里对本节点进行了限制，只有场景管理命令为true，才让本节点生效

  // check for avoidance mode, additionally
  if (enable_avoidance_)
  {
    initialized = (initialized && (current_velocity_initialized_ && costmap_initialized_ && activate_scenario_));  // 同理
  }

  return initialized;
}

// A*规划
bool AstarAvoid::planAvoidWaypoints(int& end_of_avoid_index)
{
  bool found_path = false;
  int closest_waypoint_index = getLocalClosestWaypoint(avoid_waypoints_, current_pose_global_.pose, closest_search_size_);

  // update goal pose incrementally and execute A* search
  for (int i = search_waypoints_delta_; i < static_cast<int>(search_waypoints_size_); i += search_waypoints_delta_)
  {
    // update goal index
    int goal_waypoint_index = closest_waypoint_index + obstacle_waypoint_index_ + i;
    if (goal_waypoint_index >= static_cast<int>(avoid_waypoints_.waypoints.size()))
    {
      break;
    }

    // update goal pose
    goal_pose_global_ = avoid_waypoints_.waypoints[goal_waypoint_index].pose;
    goal_pose_local_.header = costmap_.header;
    goal_pose_local_.pose = transformPose(goal_pose_global_.pose,
                                          getTransform(costmap_.header.frame_id, goal_pose_global_.header.frame_id));

    // initialize costmap for A* search
    astar_.initialize(costmap_);

    // execute astar search
    // ros::WallTime start = ros::WallTime::now();
    // 核心和全局A*一样，也是调用AstarSearch类中的A*搜索操作
    found_path = astar_.makePlan(current_pose_local_.pose, goal_pose_local_.pose);
    // ros::WallTime end = ros::WallTime::now();

    static ros::Publisher pub = nh_.advertise<nav_msgs::Path>("debug", 1, true);

    // ROS_INFO("Astar planning: %f [s], at index = %d", (end - start).toSec(), goal_waypoint_index);

    if (found_path)
    {
      pub.publish(astar_.getPath());
      end_of_avoid_index = goal_waypoint_index;
      // 和全局A*不同的地方，astar_navi会把规划出的轨迹直接发送出去
      // 而在本节点astar_avoid中，只会把这一段避障路径merge到之前的轨迹中，然后发送出去
      mergeAvoidWaypoints(astar_.getPath(), end_of_avoid_index);
      if (avoid_waypoints_.waypoints.size() > 0)
      {
        ROS_INFO("Found GOAL at index = %d", goal_waypoint_index);
        astar_.reset();
        return true;
      }
      else
      {
        found_path = false;
      }
    }
    astar_.reset();
  }

  ROS_ERROR("Can't find goal...");
  return false;
}

/*
  以end_of_avoid_index这个索引为界，这个索引 表示避障路径的终点，即避障路径的最后一个点

  在这个index之前的waypoints使用astar规划出的路径 - astar_.getPath()；

  在这个index之后的waypoints还是基于"/base_waypoints" ，即全局轨迹
*/
void AstarAvoid::mergeAvoidWaypoints(const nav_msgs::Path& path, int& end_of_avoid_index)
{
  autoware_msgs::Lane current_waypoints = avoid_waypoints_;

  // reset
  std::lock_guard<std::mutex> lock(mutex_);
  avoid_waypoints_.waypoints.clear();

  // add waypoints before start index
  int closest_waypoint_index = getLocalClosestWaypoint(current_waypoints, current_pose_global_.pose, closest_search_size_);
  for (int i = 0; i < closest_waypoint_index; ++i)
  {
    avoid_waypoints_.waypoints.push_back(current_waypoints.waypoints.at(i));
  }

  // set waypoints for avoiding
  for (const auto& pose : path.poses)
  {
    autoware_msgs::Waypoint wp;
    wp.pose.header = avoid_waypoints_.header;
    wp.pose.pose = transformPose(pose.pose, getTransform(avoid_waypoints_.header.frame_id, pose.header.frame_id));
    wp.pose.pose.position.z = current_pose_global_.pose.position.z;  // height = const
    wp.twist.twist.linear.x = avoid_waypoints_velocity_ / 3.6;       // velocity = const
    avoid_waypoints_.waypoints.push_back(wp);
  }

  // add waypoints after goal index
  for (int i = end_of_avoid_index; i < static_cast<int>(current_waypoints.waypoints.size()); ++i)
  {
    avoid_waypoints_.waypoints.push_back(current_waypoints.waypoints.at(i));
  }

  // update index for merged waypoints
  end_of_avoid_index = closest_waypoint_index + path.poses.size();
}

void AstarAvoid::publishWaypoints()
{
  autoware_msgs::Lane current_waypoints;

  while (!terminate_thread_)
  {
    // select waypoints
    /*
      如果是在正常行驶的状态，则现在的路径点current_waypoints就是base_waypoints_；
      如果是在避障状态，那么现在的路径点current_waypoints就是avoid_waypoints_；
    */
    switch (state_)
    {
      case AstarAvoid::STATE::RELAYING:
        current_waypoints = base_waypoints_;
        break;
      case AstarAvoid::STATE::STOPPING:
        // do nothing, keep current waypoints
        break;
      case AstarAvoid::STATE::PLANNING:
        // do nothing, keep current waypoints
        break;
      case AstarAvoid::STATE::AVOIDING:
        current_waypoints = avoid_waypoints_;
        break;
      default:
        current_waypoints = base_waypoints_;
        break;
    }

    autoware_msgs::Lane safety_waypoints;
    safety_waypoints.header = current_waypoints.header;
    safety_waypoints.increment = current_waypoints.increment;

    // int loc_index = getLocalClosestWaypoint(current_waypoints, current_pose_global_.pose, closest_search_size_);
    // std::cout<<"当前定位索引： "<<loc_index<<std::endl;
    for (int i = 0; i < safety_waypoints_size_; ++i)
    {
      int index = getLocalClosestWaypoint(current_waypoints, current_pose_global_.pose, closest_search_size_) + i;
      // int index = loc_index + i;
      if (index < 0 || static_cast<int>(current_waypoints.waypoints.size()) <= index)
      {
        break;
      }
      const autoware_msgs::Waypoint& wp = current_waypoints.waypoints[index];
      safety_waypoints.waypoints.push_back(wp);
    }

    if (safety_waypoints.waypoints.size() > 0)
    {
      safety_waypoints_pub_.publish(safety_waypoints);
    }

    rate_->sleep();
  }
}

tf::Transform AstarAvoid::getTransform(const std::string& from, const std::string& to)
{
  tf::StampedTransform stf;
  try
  {
    tf_listener_.lookupTransform(from, to, ros::Time(0), stf);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  return stf;
}

int AstarAvoid::getLocalClosestWaypoint(const autoware_msgs::Lane& waypoints, const geometry_msgs::Pose& pose, const int& search_size)
{
  static autoware_msgs::Lane local_waypoints;  // around self-vehicle
  const int prev_index = closest_local_index_;

  // search in all waypoints if lane_select judges you're not on waypoints
  if (closest_local_index_ == -1)
  {
    closest_local_index_ = getClosestWaypoint(waypoints, pose);
  }
  // search in limited area based on prev_index
  else
  {
    // get neighborhood waypoints around prev_index
    int start_index = std::max(0, prev_index - search_size / 2);
    int end_index = std::min(prev_index + search_size / 2, (int)waypoints.waypoints.size());
    auto start_itr = waypoints.waypoints.begin() + start_index;
    auto end_itr = waypoints.waypoints.begin() + end_index;
    local_waypoints.waypoints = std::vector<autoware_msgs::Waypoint>(start_itr, end_itr);

    // get closest waypoint in neighborhood waypoints
    int llocal_index = getClosestWaypoint(local_waypoints, pose);
    if(llocal_index != -1)
         closest_local_index_ = start_index + getClosestWaypoint(local_waypoints, pose);
    else
    {
         closest_local_index_ = prev_index;
         std::cout<<"the final index: "<<closest_local_index_<<std::endl;
    }


    // closest_local_index_ = start_index + getClosestWaypoint(local_waypoints, pose);
  }

  return closest_local_index_;
}



