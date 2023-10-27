#include "astar_global_path_presolve/pathPreSolve_core.h"

/*
  本节点接收来自astar_navi、lane_rule、lane_select的/base_waypoints（可以看作是全局轨迹）
  经过本节点的切分操作后，将/base_waypoints_cut送到astar_avoid进行局部规划
*/

pathPreSolve::pathPreSolve()
{
  // 参数 初始化
  nh_.getParam("/pathPreSolve/arrived_distance_m", arrived_distance_m);
  nh_.getParam("/pathPreSolve/stopped_velocity_mps", stopped_velocity_mps);

  // 订阅必要的话题
  current_pose_sub_ = nh_.subscribe("current_pose", 1, &pathPreSolve::currentPoseCallback, this);
  current_velocity_sub_ = nh_.subscribe("current_velocity", 1, &pathPreSolve::currentVelocityCallback, this);
  base_waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &pathPreSolve::baseWaypointsCallback, this);  // 来自于lane_select
  closest_waypoint_sub_ = nh_.subscribe("closest_waypoint", 1, &pathPreSolve::closestWaypointCallback, this);
  sub_scenario_ = nh_.subscribe("scenario_manager/scenario_cmd",1,&pathPreSolve::callbackGetScenario, this);

  // 发布 轨迹 和 换挡标志位
  base_waypoints_cut_pub_ = nh_.advertise<autoware_msgs::Lane>("base_waypoints_cut",1);
  // 在切换 前进轨迹 和 倒车轨迹 那一刻发布的标志位，用于告诉pix底盘可以进行换挡操作
  vehicle_status_pub_ = nh_.advertise<autoware_msgs::DrivingStatus>("driving_status",1);

  // 向 scenario_manager 发送是否结束。
  pub_ScenarioEnded_ = nh_.advertise<std_msgs::Bool>("scenario_ended",1);

  closest_local_index_ = -1;
  // 因为最近点是在分段轨迹上找的，然后最近点索引 + offset就是最近点在全局轨迹上的索引
  offset = 1;  // offset是当前所在分段的第一个点在全局轨迹上的索引

  // 是否已经走到 最后一段 子轨迹了
  is_completed_ = false;

  activate_scenario_ = false;

  is_ended_published = false;
}

pathPreSolve::~pathPreSolve()
{

}

void pathPreSolve::callbackGetScenario(const autoware_msgs::ScenarioCmd &msg){
  if(msg.scenario == 1 || msg.scenario == 2){ // parking scenario
    activate_scenario_ = true;
  }
  else if(msg.mode == 1){ 
    activate_scenario_ = false;
  }

  else if(msg.mode == 0)
    activate_scenario_ = true;
}

void pathPreSolve::closestWaypointCallback(const std_msgs::Int32& msg)
{
  closest_waypoint_index_ = msg.data;

  closest_waypoint_initialized_ = true;
}

void pathPreSolve::currentPoseCallback(const geometry_msgs::PoseStamped &msg){
  current_pose_ = msg;
  current_pose_initialized_ = true;

}
   
void pathPreSolve::currentVelocityCallback(const geometry_msgs::TwistStamped &msg){
  current_velocity_ = msg;
  current_velocity_initialized_ = true;
}
  
// 最重要的处理waypoint的一个回调函数，本节点的所有处理都在这里
void pathPreSolve::baseWaypointsCallback(const autoware_msgs::Lane &msg){

  // 如果astar_navi规划失败，会发布 只包含一个速度为零点 的停车轨迹，如果全局规划失败，就不处理了，直接return掉
  if (msg.waypoints.size() == 1 && msg.waypoints.at(0).twist.twist.linear.x == 0) {
    ROS_INFO("astar_navi fail! base_waypoints is empty");
    // std::cout << "astar_navi fail! base_waypoints is empty" << std::endl;
    vehicle_status_pub_.publish(prev_end_point_status);  // 再发布一次之前的档位信息，保持车辆平稳运行！
    // ROS_INFO_STREAM("prev_end_point_status: " << prev_end_point_status.data);
    return;  // 如果astar_navi规划失败，那path_presolve也不去处理了，直接return掉，让下游的astar_avoid保持上次的切分轨迹行驶
  }

  if (!is_first_update_gear) {
    // 如果全局轨迹发生变化，那么需要 更新一些变量
    if (trajectory_.waypoints.size() != msg.waypoints.size()) {
      ROS_INFO("new trajectory from astar_navi!");
      // std::cout << "new trajectory from astar_navi!" << std::endl;
      prev_target_index_ = 0;
      target_index_ = 0;
      is_completed_ = false;  // 重置 是否完成的标志位
      is_first_update_gear = true;
      is_ended_published = false;
    }
    // 下面else其实也是有新的轨迹。只是考虑的特殊情况，就是恰好两次的全局轨迹长度一样，那就判断最后一个点的信息是否不一样了
    else {
        if (trajectory_.waypoints.back().pose.pose.position.x != msg.waypoints.back().pose.pose.position.x ||
          trajectory_.waypoints.back().pose.pose.position.y != msg.waypoints.back().pose.pose.position.y ||
          trajectory_.waypoints.back().pose.pose.position.z != msg.waypoints.back().pose.pose.position.z) {

            // 如果全局轨迹发生变化，那么需要 更新一些变量
            ROS_INFO("new trajectory from astar_navi!");
            // std::cout << "new trajectory from astar_navi!" << std::endl;
            prev_target_index_ = 0;
            target_index_ = 0;
            is_completed_ = false;  // 重置 是否完成的标志位
            is_first_update_gear = true;
            is_ended_published = false;
        }
    }
  }

  trajectory_ = msg;
  reversing_indices_ = getReversingIndices(trajectory_);  // 如果要重规划，那倒车点索引 就要根据 全局轨迹变化 而实时更新

  // 如果是第一次更新档位，那么需要先发布一次档位信息，保证车辆平稳运行
  if (is_first_update_gear) {
    // 初始的档位不一定是0，需要根据实际情况修改，例如直接倒车呢，那初始就应该是1。所以这里应该从 全局轨迹 中读取 第一个点的 速度信息，通过正负获取档位
    end_point_status.vehicle_status.data = trajectory_.waypoints.at(0).twist.twist.linear.x >= 0 ? 0 : 1;  // FIXME:取等号，是因为录制轨迹的第一个点速度会是0。
    vehicle_status_pub_.publish(end_point_status);  // 第一段的 档位 也发出去
    // ROS_INFO_STREAM("end_point_status: " << end_point_status.data);

    // 由于轨迹切分放在了 档位切换的后面，所以在整个程序的第一次运行时，需要先切分一次轨迹
    updateTargetIndex();
    partial_trajectory_ = getPartialTrajectory(trajectory_, prev_target_index_, target_index_);

    is_first_update_gear = false;
  }

  // 先确定current_pose和current_velocity的topic 在各自的回调中被成功订阅
  // 再额外判断一下，现在是否处于astar场景中，activate_scenario_为true，才能激活astar相关节点的处理
  if(current_pose_initialized_ && current_velocity_initialized_ && activate_scenario_){

    // Update partial trajectory
    resetVelocity(trajectory_);  // 设置 前进和后退的速度 为定值
    std::cout << "reversing_indices size: " << reversing_indices_.size() << std::endl;

    // 由于astar_navi如果二次规划发布出全局轨迹，会导致本节点下updateTargetIndex()中的target_index不能即时更新，而报错索引越界 导致中断节点
    // 使用try来捕捉异常，异常就代表有新的全局轨迹进来，就对两个target index重新初始化
    // try {
    //   updateTargetIndex();  // 更新子目标点的索引，如果接下来有 倒车点，那倒车点就是下一个子目标点；否则就是整段轨迹的最后一个索引
    // }
    // catch (const std::exception& e) {
    //   std::cout << e.what() << std::endl;
    //   prev_target_index_ = 0;
    //   target_index_ = 0;
    //   is_completed_ = false;  // 重置 是否完成的标志位
    //   end_point_status.data = trajectory_.waypoints.at(0).twist.twist.linear.x > 0 ? 0 : 1;  // 重置档位
    // }

   
    // judge_index是当前子轨迹上车辆的索引。
    int judge_index = NewgetClosestWaypoint(partial_trajectory_, current_pose_.pose);  // NewgetClosestWaypoint()主要解决了当child trajectory 只有1个点时查找cloest_waypoint失败的问题

    // 更新子目标 放在 换挡标志位之前，保证在换挡时，is_completed_已经被更新
    updateTargetIndex();  // 更新子目标点的索引，如果接下来有 倒车点，那倒车点就是下一个子目标点；否则就是整段轨迹的最后一个索引

    // 如果还没走到 最后一段 子轨迹（is_completed_表示走到最后一段子轨迹的最后两个点）
    if (is_completed_ == false) {
        // 如果车辆走到当前子轨迹的最后两个点，那就要换挡了
        if (judge_index >= int(partial_trajectory_.waypoints.size()) - 2) {
            // 在进入到当前子轨迹的最后两个点，就要马上更新下一段子轨迹，并依据下一段子轨迹的速度给档位标志位赋值
            partial_trajectory_ = getPartialTrajectory(trajectory_, prev_target_index_, target_index_);
            end_point_status.vehicle_status.data = partial_trajectory_.waypoints.at(0).twist.twist.linear.x >= 0 ? 0 : 1;
            // 发布 底盘所需要的换挡标志位
            vehicle_status_pub_.publish(end_point_status);
            // ROS_INFO_STREAM("end_point_status: " << end_point_status.data);
            prev_end_point_status = end_point_status;
        }
    }
    // is_completed为true，表示现在处于 最后一段 子轨迹了，可以适时发布最终的停车指令
    else {
        // 加等号是因为 避免 最后一段 只有两个点的情况，此时judge_index会等于size()-2
        if (judge_index >= int(partial_trajectory_.waypoints.size()) - 2) {   // 这里的size()返回的是size_t, 如果相减为负数, 很可能是一个巨大正数                        
            if(!is_ended_published){
              // 发给控制的 停车标志位
              end_point_status.vehicle_status.data = 2;  // 在mpc中做了停车处理
              // 这里发一次VehicleCleaningStatus，只设置了vehicle_status字段，其他字段都是默认值（例如is_garbaging是false）
              vehicle_status_pub_.publish(end_point_status);
              prev_end_point_status = end_point_status;
              // ROS_INFO_STREAM("end_point_status: " << end_point_status.data);

              std_msgs::Bool ended;
              ended.data = true;
              // astar场景完成标志位，发送给场景管理节点，准备切换下一个场景
              pub_ScenarioEnded_.publish(ended);
              is_ended_published = true;

              // 最终停车标志位
              end_point_status.vehicle_status.data = 2;  // 在mpc中做了停车处理
              // vehicle_status_pub_.publish(end_point_status);
              // ROS_INFO_STREAM("end_point_status: " << end_point_status.data);
            }
        }
    }


    // 将当前的子轨迹 发布出去 供 A*（astar_avoid）用作baseway_points_，即 RELAYING状态跟踪的轨迹
    base_waypoints_cut_pub_.publish(partial_trajectory_);

    std::cout << "is_completed: " << is_completed_ << std::endl;

    std::cout << "vehicle status: " << end_point_status.vehicle_status.data << std::endl;  // 打印 一个 表示 档位信息的 话题，0表示前进，1表示倒退，2表示最终停车

    std::cout << "whole trajectory waypoints size: " << trajectory_.waypoints.size() << std::endl;  // 打印 整段 轨迹 的长度

    std::cout << "current child trajectory waypoints size: " << partial_trajectory_.waypoints.size() << std::endl;  // 打印当前 子轨迹 的长度

    std::cout << "vehicle's target index in global trajectory: " << target_index_ << std::endl;  // 打印当前 的目标索引（在全局轨迹下的）

    std::cout << "vehicle's closest index in child trajectory: " << judge_index << std::endl;  // 打印车辆 在当前子轨迹 的最近索引
    
    std::cout << "----------------------------" << std::endl;
  }

}

void pathPreSolve::resetVelocity(autoware_msgs::Lane &original_waypoints){
 std::vector<autoware_msgs::Waypoint> &wayPoints = original_waypoints.waypoints;
 for (auto &point : wayPoints){
  if (point.twist.twist.linear.x > 0){
    point.twist.twist.linear.x = 1.0;  // 前进的速度永远是1m/s
  }
  else if (point.twist.twist.linear.x < -0.1){
    point.twist.twist.linear.x = -0.5;  // 倒车的速度永远是-0.5m/s
  } 
 }
 return;
}


void pathPreSolve::updateTargetIndex() {

  // 找到当前车辆位置 在 当前子轨迹中 最近的一个waypoint
  int current_local_index = NewgetClosestWaypoint(partial_trajectory_, current_pose_.pose);

  // 把距离判断的方式 改称 索引判断
  if ((current_local_index >= int(partial_trajectory_.waypoints.size()) - 2) || is_first_update_target) {
    std::cout << "it's time to check the next child trajectory!" << std::endl;

    // reversing_indices_记录了所以需要颠倒方向的倒车点，getNextTargetIndex()函数返回下一个目标点的索引（如果遇到了倒车点，那就把下一个目标点设置为下一个倒车点）
    const auto new_target_index =
      getNextTargetIndex(trajectory_.waypoints.size(), reversing_indices_, target_index_);

    std::cout << "new target index: " << new_target_index << std::endl;
    std::cout << "target index: " << target_index_ << std::endl;

    // 这里如果new_target_index == target_index_，表明上面的getNextTargetIndex()的过程中，接下来没有新的倒车点了，意思是可以直接不切换正负方向可以直接开到 整段的终点
    if (new_target_index == target_index_) {
      ROS_INFO("reverse planning completed, directly go to the end of the trajectory!");
      // 额外发布一个is_completed_的话题，表示 后续不需要再 大转向了，走完当前子轨迹即可
      is_completed_ = true;  // 这个变量只会在：走到最后一段子轨迹的最后两个点内，会置成true
    }
    else {
      // 否则，需要切换到下一段 子轨迹
      ROS_INFO("switch to next child trajectory!");
      prev_target_index_ = target_index_;
      
      target_index_ =
        getNextTargetIndex(trajectory_.waypoints.size(), reversing_indices_, target_index_);
    }

    is_first_update_target = false;  // 只有在节点更启动时，is_first_update_target会在类初始化时 置成true，然后后面这个值都应该时false
  }
}

// 从轨迹中提前获取倒车点（方向切换点）
std::vector<size_t> pathPreSolve::getReversingIndices(const autoware_msgs::Lane & trajectory)
{
  std::vector<size_t> indices;

  // 遍历到倒数第二个点，是因为下面判断速度的时候，是前后点速度相乘；否则i+1会越界
  for (size_t i = 0; i < trajectory.waypoints.size() - 1; ++i) {
    // 利用前后两点的速度 乘积是否小于0 来判断 该点 是否为方向切换点（简称为“倒车点”)
    if (trajectory.waypoints.at(i).twist.twist.linear.x * trajectory.waypoints.at(i + 1).twist.twist.linear.x <0) {
      indices.push_back(i);
    }
  }

  return indices;
}

// 这里体现了 整个倒车算法的一个思路，就是将整条轨迹 按照 倒车点（方向正负改变的点） 截成 一段段，每次完成一段的规划，再进行下一段
size_t pathPreSolve::getNextTargetIndex(
  const size_t trajectory_size, const std::vector<size_t> & reversing_indices,
  const size_t current_target_index)
{
  if (!reversing_indices.empty()) {  // 如果有 开始方向反转的倒车点，那么下一个目标点就是倒车点
    for (const auto reversing_index : reversing_indices) {
      // reversing_index和target_index都是在全局轨迹上的索引（而不是子轨迹）
      if (reversing_index > current_target_index) {  // 如果下一个倒车点索引 大于 当前目标点索引，那么下一个目标点就是倒车点
        return reversing_index;
      }
    }
  }

  return trajectory_size - 1;  // 如果剩余轨迹没有倒车点了，就直接返回 整条轨迹的最后一个索引
}


// 从整条轨迹中  截取指定索引范围的一段轨迹
autoware_msgs::Lane pathPreSolve::getPartialTrajectory(const autoware_msgs::Lane & trajectory, const size_t start_index, const size_t end_index)
{
  autoware_msgs::Lane partial_trajectory;
  partial_trajectory.increment = 1; // parking lane

  partial_trajectory.waypoints.reserve(trajectory.waypoints.size());  // 预设vector的大小
  for (size_t i = start_index; i <= end_index; ++i) {
    partial_trajectory.waypoints.push_back(trajectory.waypoints.at(i));
  }

  // 如果这条轨迹只有一个点, 为了避免getClosetWaypoint返回-1的问题, 人工加一个虚拟点
  if (partial_trajectory.waypoints.size() == 1) {
    autoware_msgs::Waypoint waypoint(partial_trajectory.waypoints.front());
    waypoint.pose.pose.position.x += 0.1; 
    waypoint.pose.pose.position.y += 0.1;
    partial_trajectory.waypoints.push_back(waypoint);
  }

  // Modify velocity at start/end point
  if (partial_trajectory.waypoints.size() >= 2) {  // 第一个轨迹点的速度设置为第二个轨迹点的速度
    partial_trajectory.waypoints.front().twist.twist.linear.x =
      partial_trajectory.waypoints.at(1).twist.twist.linear.x;
  }
  if (!partial_trajectory.waypoints.empty()) {  // 最后一个轨迹点的速度设置为0
    partial_trajectory.waypoints.back().twist.twist.linear.x = 0;
  }


  return partial_trajectory;
}



void pathPreSolve::MainLoop(){
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}