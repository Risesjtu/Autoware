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

#include "freespace_planner/astar_navi.h"

// astar_navi以A*为规划方法，可以提供一条 全局轨迹！供后续如astar_avoid等 去局部避障使用 

AstarNavi::AstarNavi() : nh_(), private_nh_("~")
{
  activate_scenario = false;
  private_nh_.param<double>("waypoints_velocity", waypoints_velocity_, 5.0);
  private_nh_.param<double>("update_rate", update_rate_, 1.0);
  private_nh_.param<bool>("use_back", use_back_, false);

  lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>("lane_waypoints_array_astar", 1, true);

  costmap_sub_ = nh_.subscribe("costmap", 1, &AstarNavi::costmapCallback, this);
  current_pose_sub_ = nh_.subscribe("current_pose", 1, &AstarNavi::currentPoseCallback, this);
  goal_pose_sub_ = nh_.subscribe("move_base_simple/goal_fine", 1, &AstarNavi::goalPoseCallback, this);
  sub_scenario = nh_.subscribe("scenario_manager/scenario_cmd",1,&AstarNavi::callbackGetScenario,this);

  costmap_initialized_ = false;
  current_pose_initialized_ = false;
  goal_pose_initialized_ = false;

  // for interpolation

  WaypointReplannerConfig temp_config;
  temp_config.radius_thresh = 20;
  temp_config.radius_min = 6;
  temp_config.radius_inf = 10 * temp_config.radius_thresh;
  temp_config.resample_interval = 0.2;
  temp_config.lookup_crv_width = 3;
  replanner_.updateConfig(temp_config);

#ifdef VISUALIZATION
  vehicle.set_topic("vehicle");
  vehicle.set_type(vehicle.POLYGON);

  path_original.set_topic("original_path");
  path_original.set_type(path_original.CURVE);

  path_resample.set_topic("resample_path");
  path_resample.set_type(path_resample.CURVE);
#endif
  
}

AstarNavi::~AstarNavi()
{

}

void AstarNavi::costmapCallback(const nav_msgs::OccupancyGrid& msg)
{
  costmap_ = msg;
  tf::poseMsgToTF(costmap_.info.origin, local2costmap_);

  costmap_initialized_ = true;
}

void AstarNavi::currentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  if (!costmap_initialized_)
  {
    return;
  }

  current_pose_global_ = msg;
  // 转一下frame_id，将costmap和车辆当前位姿的frame_id统一，将车辆当前位姿转换到costmap的坐标系下
  current_pose_local_.pose = transformPose(
      current_pose_global_.pose, getTransform(costmap_.header.frame_id, current_pose_global_.header.frame_id));
  current_pose_local_.header.frame_id = costmap_.header.frame_id;
  current_pose_local_.header.stamp = current_pose_global_.header.stamp;

  current_pose_initialized_ = true;
}

void AstarNavi::goalPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  // if (!costmap_initialized_)
  // {
  //   return;
  // }

  goal_pose_global_ = msg;
  // goal_pose_local_.pose =
  //     transformPose(goal_pose_global_.pose, getTransform(costmap_.header.frame_id, goal_pose_global_.header.frame_id));
  // goal_pose_local_.header.frame_id = costmap_.header.frame_id;
  // goal_pose_local_.header.stamp = goal_pose_global_.header.stamp;

  goal_pose_initialized_ = true;

  // ROS_INFO_STREAM("Subscribed goal pose and transform from " << msg.header.frame_id << " to "
  //                                                            << goal_pose_local_.header.frame_id << "\n"
  //                                                            << goal_pose_local_.pose);
  // khy add:增加一个标志位，只有订阅到新的Goal（来自Rviz中勾选，或者直接发布对应的话题），进入到这个回调，才把标志位置true，才能在下面的主循环 进入 if进行 publishWaypoints()，即有新的goal，才发布新的全局轨迹                                                        
  new_goal_ = true;
}

void AstarNavi::convertFrameId(){
  if (!costmap_initialized_)
  {
    return;
  }
  goal_pose_local_.pose =
      transformPose(goal_pose_global_.pose, getTransform(costmap_.header.frame_id, goal_pose_global_.header.frame_id));
  goal_pose_local_.header.frame_id = costmap_.header.frame_id;
  goal_pose_local_.header.stamp = goal_pose_global_.header.stamp;

  // goal_pose_initialized_ = true;

  ROS_INFO_STREAM("Subscribed goal pose and transform from " << "velodyne" << " to "
                                                             << goal_pose_local_.header.frame_id << "\n"
                                                             << goal_pose_local_.pose);
                                                             
}



tf::Transform AstarNavi::getTransform(const std::string& from, const std::string& to)
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


void AstarNavi::callbackGetScenario(const autoware_msgs::ScenarioCmd &msg){
  if(msg.scenario == 1){
    activate_scenario = true;
    goalPoseCallback(msg.goal);
    ROS_INFO("receieve parking scenario in astar_navi!");
  }
  else if(msg.mode == 1){
    activate_scenario = false;
    ROS_INFO("receive exclusive scenario order, shutdown current scenario!");
  }

  else if(msg.mode == 0)
    activate_scenario = true;
}

void AstarNavi::run()
{
  ros::Rate rate(update_rate_);

  nav_msgs::Path empty_path;
  empty_path.header.stamp = ros::Time::now();
  empty_path.header.frame_id = costmap_.header.frame_id;

  while (ros::ok())
  {
    ros::spinOnce();

    if (!costmap_initialized_ || !current_pose_initialized_ || !goal_pose_initialized_ || !activate_scenario)
    // if (!costmap_initialized_ || !current_pose_initialized_ || !goal_pose_initialized_ )
    {
      rate.sleep();
      continue;
    }

    // initialize vector for A* search, this runs only once
    astar_.initialize(costmap_);
    convertFrameId();

    // update local goal pose
    // khy fix:不需要在主循环中一直更新goal，有话题来触发回调函数自然就会更新goal。放在这里反而失去了回调的作用
    // goalPoseCallback(goal_pose_global_);

    // execute astar search
    ros::WallTime start = ros::WallTime::now();
    bool result = astar_.makePlan(current_pose_local_.pose, goal_pose_local_.pose);
    // ROS_INFO_STREAM("original planning path size: " << astar_.getPath().poses.size());
    ros::WallTime end = ros::WallTime::now();

    ROS_INFO("Astar planning: %f [s]", (end - start).toSec());

    // result就是A*返回的 是否规划成功的 标志位
    if (new_goal_) {
      if (result)
      {
        ROS_INFO("Found GOAL!");

        // khy add:去除离群点
        std::pair<nav_msgs::Path, std::vector<bool>> path_vel = removeOutliers(astar_.getPath(), astar_.velocity_flag_);
        nav_msgs::Path remove_outliers_path = path_vel.first;
        std::vector<bool> vel_flag = path_vel.second;
        // ROS_INFO_STREAM("after outliers, path size: " << remove_outliers_path.poses.size());


        // publishWaypoints()函数中，将astar_search()规划出来的 路径 发布出去，同时将速度的方向信息velocity_flag用起来
        publishWaypoints(remove_outliers_path, waypoints_velocity_, vel_flag);  // 使用去除离群点后的轨迹
        // publishWaypoints(astar_.getPath(), waypoints_velocity_, astar_.velocity_flag_);

        // 原先下面的break是注释掉的，这样 astar_navi节点 就会 随着 当前车辆 位置变动，也会不断改变规划出的全局轨迹。
        // 但是这样其实不好！更新的太频繁，不利于后续的跟踪，或者倒车时轨迹切割节点也暂时不兼容一直更新全局轨迹
        // 所以修改成：一旦规划出一条全局轨迹，就直接结束astar_navi
        // break;

        // 最新修改，订阅到的goal发生变化了，就发布一次新的全局轨迹
        new_goal_ = false;
      }
      else
      {
        ROS_INFO("Can't find goal...");
        publishStopWaypoints();
      }
    }
    

    astar_.reset();
    rate.sleep();
  }
}

// 将规划出来的 路径 发布出去
// 修改这个函数实现，多增加一个参数：velocity_flag_，即把astar_search中保存的 路径点速度正负信息，在这里用起来，给最终的 发布轨迹 赋值 分段的速度正负
// (原先也有速度正负赋值，但是只能给一整条轨迹赋值一样的符号，全为正或全为负)
void AstarNavi::publishWaypoints(const nav_msgs::Path& path, const double& velocity, const std::vector<bool>& velocity_flag)
{
  autoware_msgs::Lane lane;
  lane.header.frame_id = "map";
  lane.header.stamp = path.header.stamp;
  lane.increment = 0;
  // bool reverse_flag = false;
  int points_num = path.poses.size();
  int index;

  // FIXME：废弃使用原先的reverse_flag计算方式，改用velocity_flag_去赋值reverse_flag，即把astar_search中保存的 路径点速度正负信息，在这里用起来，给最终的 发布轨迹 赋值 分段的速度正负
  /*
    nav_msgs::Path是ROS格式信息，其包含header和poses两个属性，其中poses是geometry_msgs/PoseStamped
  */
  for (int i = 0; i < path.poses.size(); i++) {
    geometry_msgs::PoseStamped pose = path.poses[i];
    bool reverse_flag = velocity_flag[i];
    
    autoware_msgs::Waypoint wp;
    wp.pose.header = lane.header;
    wp.pose.pose = transformPose(pose.pose, getTransform(lane.header.frame_id, pose.header.frame_id));
    wp.pose.pose.position.z = current_pose_global_.pose.position.z;  // height = const

    // 根据AstarNode->back的定义，true表示后退，false表示前进。velocity_flag就是存了back的值
    if(reverse_flag)
    {
      wp.twist.twist.linear.x = velocity / 3.6*-1.0;
    }
    else{
      wp.twist.twist.linear.x = velocity / 3.6;  // velocity = const
    }
    lane.waypoints.push_back(wp);
  }

   
  // resample 
  autoware_msgs::Lane resample_lane;
  resample_lane.header.frame_id = "map";
  resample_lane.header.stamp = path.header.stamp;
  resample_lane.increment = 0;
  resampleAstarPath(lane,resample_lane);


#ifdef VISUALIZATION
  curve_sender(sender,path_original,lane);
  curve_sender(sender,path_resample,resample_lane);
#endif
  
   
  autoware_msgs::LaneArray lane_array;
  // lane_array.lanes.push_back(lane);
  lane_array.lanes.push_back(resample_lane);  // 增密轨迹点
  lane_pub_.publish(lane_array);

}

// 用于 发布 一个速度为0的 点，接在当前位姿轨迹后面，用于停车（A*遇障碍物后的重规划，需要先停车）
void AstarNavi::publishStopWaypoints()
{
  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;  // stop path
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = current_pose_global_.header.frame_id;
  pose.pose = current_pose_global_.pose;
  path.poses.push_back(pose);
  // 发布一个 只包含一个速度为零的点 的 轨迹，用于停车
  publishWaypoints(path, 0.0, std::vector<bool>(1, false));  // khy fix:配合修改，publishWaypoints()需要多一个vector参数
}

void AstarNavi::resampleAstarPath(const autoware_msgs::Lane &original_lane, autoware_msgs::Lane &resample_lane){
  
  std::vector<size_t> indices;

  for (size_t i = 0; i < original_lane.waypoints.size() - 1; ++i) {
    if (original_lane.waypoints.at(i).twist.twist.linear.x * original_lane.waypoints.at(i + 1).twist.twist.linear.x <0) {
      indices.push_back(i);
    }
  }
  indices.push_back(original_lane.waypoints.size() - 1);

  size_t pre_index = 0;

  for(size_t j = 0;j<indices.size();j++){
    size_t curr_index = indices[j];
    autoware_msgs::Lane partial_lane;
    partial_lane.waypoints.insert(partial_lane.waypoints.begin(), 
                                   original_lane.waypoints.begin()+pre_index,
                                  original_lane.waypoints.begin()+curr_index+1);
    replanner_.replanLaneWaypoint(partial_lane);
    resample_lane.waypoints.insert(resample_lane.waypoints.end(),partial_lane.waypoints.begin(),partial_lane.waypoints.end());
    pre_index = curr_index+1;
  }

}


#ifdef VISUALIZATION
void AstarNavi::curve_sender(Sender &sender, visualization::Frame &msg, autoware_msgs::Lane &lane)
{
  msg.clear_lines();
  visualization::BaseLine *line_ptr = msg.add_lines();
  for(auto &lane_point:lane.waypoints){
    visualization::Point *point = line_ptr->add_points();
    point->set_x(lane_point.pose.pose.position.x);
    point->set_y(lane_point.pose.pose.position.y);
  }
  // for(int i=0;i<10;i++){
  //     visualization::Point *point = line_ptr->add_points();
  //     point->set_x(i);
  //     point->set_y(i);
  // }
  line_ptr->set_thickness(5);
  line_ptr->set_symbol(line_ptr->o);
  sender.send(msg);
}

void AstarNavi::polygon_sender(Sender &sender, visualization::Frame &msg, geometry_msgs::PoseStamped &current_pose)
{

}

#endif

// 去除离群点
std::pair<nav_msgs::Path, std::vector<bool>> AstarNavi::removeOutliers(const nav_msgs::Path& path_, const std::vector<bool>& velocity_flag)
{
  nav_msgs::Path path = path_;
  std::vector<bool> flag = velocity_flag;

  std::vector<size_t> reversing_indices = getReversingIndices(flag);

  // ROS_INFO_STREAM("reversing index: ");
  // for (auto i : reversing_indices) {
  //   std::cout << i << std::endl;
  // }

  // 如果没有转折点(第一个点和最后一个点不算)，直接返回
  if (reversing_indices.size() == 2) {
    return std::make_pair(path, flag);
  }

  std::vector<size_t> outlier_indices;

  // 记录所有要删除的离群点的索引（某一段子轨迹 数目小于3个点 的轨迹点 算离群点）
  for (int i = 0; i < reversing_indices.size() - 1; i++) {
    if (reversing_indices[i + 1] - reversing_indices[i] < 4) {
      if (i == 0) {
        for (int j = reversing_indices[i]; j <= reversing_indices[i + 1]; j++) {
          outlier_indices.push_back(j);
        }
      }
      else {
        for (int j = reversing_indices[i] + 1; j <= reversing_indices[i + 1]; j++) {
          outlier_indices.push_back(j);
        }
      }
    }
  }


  if (outlier_indices.size() == 0)
    return std::make_pair(path, flag);

  // 在轨迹中 删除 离群点
  // ROS_INFO_STREAM("outliers index: ");
  for (int i = 0; i <= outlier_indices.size() - 1; i++) {
    // std::cout << outlier_indices[i] << std::endl;
    // 除了多加一个-i，也可以按照索引大小，倒叙删除，先删除大的索引
    path.poses.erase(path.poses.begin() + outlier_indices[i] - i);  // erase()删除一次，vector的长度就会改变，下次要删的点的索引也会变，所以要-i
    flag.erase(flag.begin() + outlier_indices[i] - i);
  }

  // ROS_INFO_STREAM("after outliers, velocity flag: ");
  // for (int i = 0; i <= flag.size() - 1; i++) {
  //   std::cout << flag[i] << std::endl;
  // }

  return std::make_pair(path, flag);
}

// 获取 轨迹中 正负速度 转折点，特别的，第一个点也算作转折点
std::vector<size_t> AstarNavi::getReversingIndices(const std::vector<bool>& velocity_flag)
{
  std::vector<size_t> indices;
  indices.push_back(0);

  // ROS_INFO_STREAM("velocity_flag: ");
  for (size_t i = 0; i < velocity_flag.size() - 1; ++i) {
    // std::cout << velocity_flag[i] << std::endl;
    // 利用前后两点的速度方向，异或是否等于1 来判断 该点 是否为方向切换点（简称为“倒车点”)
    if ((velocity_flag[i] ^ velocity_flag[i + 1]) == 1) {
      indices.push_back(i);
    }
  }
  // std::cout << velocity_flag[velocity_flag.size() - 1] << std::endl;

  indices.push_back(velocity_flag.size() - 1);  // 把最后一个点的索引也加入

  return indices;
}
