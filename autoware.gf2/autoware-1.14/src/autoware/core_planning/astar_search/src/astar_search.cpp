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

#include "astar_search/astar_search.h"

AstarSearch::AstarSearch()
{
  ros::NodeHandle private_nh_("~");

  // base configs
  private_nh_.param<bool>("use_back", use_back_, true);
  private_nh_.param<bool>("use_potential_heuristic", use_potential_heuristic_, true);
  private_nh_.param<bool>("use_wavefront_heuristic", use_wavefront_heuristic_, false);
  private_nh_.param<double>("time_limit", time_limit_, 5000.0);
  private_nh_.param<double>("shot_distance",shot_distance, 5);
  private_nh_.param<bool>("use_rs_curve",use_rs_curve_,false);

  // robot configs
  private_nh_.param<double>("robot_length", robot_length_, 4.5);
  private_nh_.param<double>("robot_width", robot_width_, 1.75);
  private_nh_.param<double>("robot_base2back", robot_base2back_, 1.0);
  private_nh_.param<double>("minimum_turning_radius", minimum_turning_radius_, 6.0);

  // search configs
  private_nh_.param<int>("theta_size", theta_size_, 48);
  private_nh_.param<double>("angle_goal_range", angle_goal_range_, 6.0);
  private_nh_.param<double>("curve_weight", curve_weight_, 1.2);
  private_nh_.param<double>("reverse_weight", reverse_weight_, 2.00);
  private_nh_.param<double>("lateral_goal_range", lateral_goal_range_, 0.5);
  private_nh_.param<double>("longitudinal_goal_range", longitudinal_goal_range_, 2.0);

  // costmap configs
  private_nh_.param<int>("obstacle_threshold", obstacle_threshold_, 100);
  private_nh_.param<double>("potential_weight", potential_weight_, 10.0);
  private_nh_.param<double>("distance_heuristic_weight", distance_heuristic_weight_, 1.0);

  createStateUpdateTable();

  rs_path_ptr_ = std::make_shared<RSPath>(minimum_turning_radius_);  // 实例化RS的类
}

AstarSearch::~AstarSearch()
{
}

// state update table for hybrid astar
// 计算不同角分辨率 所对应的 搜索位移，形成一个table，在后续的A*遍历周围节点时，直接从table中取值
/*
  由于方向角度的不同，在不同方向的位移（shift量）也是不同的。搜索过程不可能简单的上下左右几个格子这样去寻找，因为每个node还包括了姿态信息。

  这个函数相当于一个几何化的过程，提前为搜索过程预设好了每种情况的shift，后续搜索过程中只要调用就好了。
*/
void AstarSearch::createStateUpdateTable()
{
  // Vehicle moving for each angle
  state_update_table_.resize(theta_size_);  // state_update_table_是一个二维数组，第一维是角分辨率，第二维是位移量
  double dtheta = 2.0 * M_PI / theta_size_;

  // Minimum moving distance with one state update
  //     arc  = r                       * theta
  double step = minimum_turning_radius_ * dtheta;

  for (int i = 0; i < theta_size_; i++)
  {
    double theta = dtheta * i;

    // Calculate right and left circle
    // Robot moves along these circles
    double right_circle_center_x = minimum_turning_radius_ * std::sin(theta);
    double right_circle_center_y = minimum_turning_radius_ * -std::cos(theta);
    double left_circle_center_x = -right_circle_center_x;
    double left_circle_center_y = -right_circle_center_y;

    // Calculate x and y shift to next state
    NodeUpdate nu;

    // forward
    nu.shift_x = step * std::cos(theta);
    nu.shift_y = step * std::sin(theta);
    nu.rotation = 0.0;
    nu.index_theta = 0;
    nu.step = step;
    nu.curve = false;
    nu.back = false;
    state_update_table_[i].emplace_back(nu);

    // forward right
    nu.shift_x = right_circle_center_x + minimum_turning_radius_ * std::cos(M_PI_2 + theta - dtheta);
    nu.shift_y = right_circle_center_y + minimum_turning_radius_ * std::sin(M_PI_2 + theta - dtheta);
    nu.rotation = -dtheta;
    nu.index_theta = -1;
    nu.step = step;
    nu.curve = true;
    nu.back = false;
    state_update_table_[i].emplace_back(nu);

    // forward left
    // 每个方向上搜索的半径，多大的shift都已经算好了
    nu.shift_x = left_circle_center_x + minimum_turning_radius_ * std::cos(-M_PI_2 + theta + dtheta);
    nu.shift_y = left_circle_center_y + minimum_turning_radius_ * std::sin(-M_PI_2 + theta + dtheta);
    nu.rotation = dtheta;
    nu.index_theta = 1;
    nu.step = step;
    nu.curve = true;
    nu.back = false;
    state_update_table_[i].emplace_back(nu);

    if (use_back_)
    {
      // backward
      nu.shift_x = step * std::cos(theta) * -1.0;
      nu.shift_y = step * std::sin(theta) * -1.0;
      nu.rotation = 0;
      nu.index_theta = 0;
      nu.step = step;
      nu.curve = false;
      nu.back = true;
      state_update_table_[i].emplace_back(nu);

      // backward right
      nu.shift_x = right_circle_center_x + minimum_turning_radius_ * std::cos(M_PI_2 + theta + dtheta);
      nu.shift_y = right_circle_center_y + minimum_turning_radius_ * std::sin(M_PI_2 + theta + dtheta);
      nu.rotation = dtheta;
      nu.index_theta = 1;
      nu.step = step;
      nu.curve = true;
      nu.back = true;
      state_update_table_[i].emplace_back(nu);

      // backward left
      nu.shift_x = left_circle_center_x + minimum_turning_radius_ * std::cos(-1.0 * M_PI_2 + theta - dtheta);
      nu.shift_y = left_circle_center_y + minimum_turning_radius_ * std::sin(-1.0 * M_PI_2 + theta - dtheta);
      nu.rotation = dtheta * -1.0;
      nu.index_theta = -1;
      nu.step = step;
      nu.curve = true;
      nu.back = true;
      state_update_table_[i].emplace_back(nu);
    }
  }
}

// costmap的初始化
// 对costmap进行初始化，将map信息存入定义好的 nodes_ 结构体 里面，nodes_ 中的每一个node包含了astar算法中需要用到的状态量。
// 将costmap格式转换成nodes_ 的形式，即每个栅格用一个node保存。A*算法是对周围栅格进行遍历搜索，代码中就体现为利用nodes_[i][j]的索引去遍历搜索
void AstarSearch::initialize(const nav_msgs::OccupancyGrid& costmap)
{
  costmap_ = costmap;

  int height = costmap_.info.height;
  int width = costmap_.info.width;

  // size initialization
  // 首先，对每个栅格的基本尺寸做一个初始化，每个栅格用一个node表示；
  nodes_.resize(height);
  for (int i = 0; i < height; i++)
  {
    nodes_[i].resize(width);
  }
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      nodes_[i][j].resize(theta_size_);  // 每个node都包含了三维的信息：除了自身i、j编号表示的xy位置信息，还有存储的yaw角表示姿态。
    }
  }

  // cost initialization
  // 更新每个栅格的代价
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      // Index of subscribing OccupancyGrid message
      int og_index = i * width + j;
      int cost = costmap_.data[og_index];

      // hc is set to be 0 when reset()
      if (cost == 0)
      {
        continue;
      }

      // obstacle or unknown area
      // 如果某个栅格node的代价小于零，就判断为障碍物或者不可通行区域，将这个node的状态status设置为STATUS::OBS；
      if (cost < 0 || obstacle_threshold_ <= cost)
      {
        nodes_[i][j][0].status = STATUS::OBS;
      }

      // the cost more than threshold is regarded almost same as an obstacle
      // because of its very high cost
      if (use_potential_heuristic_)
      {
        nodes_[i][j][0].hc = cost * potential_weight_;
      }
    }
  }
}

// makePlan()是A*的一个入口
bool AstarSearch::makePlan(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose)
{
  if (!setStartNode(start_pose))
  {
    // ROS_WARN_STREAM("Invalid start pose");
    return false;
  }

  if (!setGoalNode(goal_pose))
  {
    // ROS_WARN_STREAM("Invalid goal pose");
    return false;
  }

  // 如果起点 和 终点 都设置成功，就开始搜索！search()中是核心操作
  return search();
}

// 和setGoalNode()差不多，只不过这里是设置起点
bool AstarSearch::setStartNode(const geometry_msgs::Pose& start_pose)
{
  // Get index of start pose
  int index_x, index_y, index_theta;
  start_pose_local_.pose = start_pose;
  poseToIndex(start_pose_local_.pose, &index_x, &index_y, &index_theta);
  SimpleNode start_sn(index_x, index_y, index_theta, 0, 0);

  // Check if start is valid
  if (isOutOfRange(index_x, index_y) || detectCollision(start_sn))
  {
    return false;
  }

  // Set start node、
  // 然后对于起点index，把它设置为第一个node，并且设置一些它的初始信息，如gc代价：0，父节点：NULL，状态：STATUS::OPEN（表示 应该 在openlist中）等等。
  AstarNode& start_node = nodes_[index_y][index_x][index_theta];
  start_node.x = start_pose_local_.pose.position.x;
  start_node.y = start_pose_local_.pose.position.y;
  start_node.theta = 2.0 * M_PI / theta_size_ * index_theta;
  start_node.gc = 0;  // 起点 到 起点 的代价肯定是零
  start_node.move_distance = 0;
  start_node.back = false;  // 这个back属性 表示了 当前点 应该是 前进 还是 后退！
  start_node.status = STATUS::OPEN;
  start_node.parent = NULL;

  // set euclidean distance heuristic cost
  // 选择欧氏距离方法计算 起点的 hc (当前点到终点距离) 
  if (!use_wavefront_heuristic_ && !use_potential_heuristic_)
  {
    start_node.hc = calcDistance(start_pose_local_.pose.position.x, start_pose_local_.pose.position.y,
                                 goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y) *
                    distance_heuristic_weight_;
  }
  // 这种计算代价的方法暂没阅读，因为use_potential_heuristic_默认是false
  else if (use_potential_heuristic_)
  {
    start_node.gc += start_node.hc;
    start_node.hc += calcDistance(start_pose_local_.pose.position.x, start_pose_local_.pose.position.y,
                                  goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y) +
                     distance_heuristic_weight_;
  }

  // Push start node to openlist
  // 最后将起始点放入openList中，作为openList中的第一个计算点
  start_sn.cost = start_node.gc + start_node.hc;
  openlist_.push(start_sn);

  return true;
}

// 终点设置函数
bool AstarSearch::setGoalNode(const geometry_msgs::Pose& goal_pose)
{
  goal_pose_local_.pose = goal_pose;
  goal_yaw_ = modifyTheta(tf::getYaw(goal_pose_local_.pose.orientation));

  // for rs path 
  goal_node_.x = goal_pose.position.x;
  goal_node_.y = goal_pose.position.y;
  goal_node_.theta = Mod2Pi(goal_yaw_);

  // Get index of goal pose
  int index_x, index_y, index_theta;
  // 将pose将转换到costmap坐标系下下，并且转换为 栅格地图中 对应 的索引
  poseToIndex(goal_pose_local_.pose, &index_x, &index_y, &index_theta);
  SimpleNode goal_sn(index_x, index_y, index_theta, 0, 0);

  // Check if goal is valid
  // 接着判断获取的终点索引是否在整个栅格图之外，或者终点设置在了障碍物上，只有都不是才能执行后续操作；
  if (isOutOfRange(index_x, index_y) || detectCollision(goal_sn))
  {
    return false;
  }

  // Calculate wavefront heuristic cost
  // 根据参数设置if (use_wavefront_heuristic_)，可以启动另一种代价计算方式。但默认是false，所以暂不了解。
  if (use_wavefront_heuristic_)
  {
    // auto start = std::chrono::system_clock::now();
    bool wavefront_result = calcWaveFrontHeuristic(goal_sn);
    // auto end = std::chrono::system_clock::now();
    // auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // std::cout << "wavefront : " << usec / 1000.0 << "[msec]" << std::endl;

    if (!wavefront_result)
    {
      // ROS_WARN("Reachable is false...");
      return false;
    }
  }

  return true;
}

// 将pose将转换到costmap坐标系下下，并且转换为 栅格地图中 对应 的索引
void AstarSearch::poseToIndex(const geometry_msgs::Pose& pose, int* index_x, int* index_y, int* index_theta)
{
  tf::Transform orig_tf;
  tf::poseMsgToTF(costmap_.info.origin, orig_tf);
  geometry_msgs::Pose pose2d = transformPose(pose, orig_tf.inverse());

  *index_x = pose2d.position.x / costmap_.info.resolution;  // 就是在initialize()中 用costmap_的信息初始化的 nodes_信息
  *index_y = pose2d.position.y / costmap_.info.resolution;

  tf::Quaternion quat;
  tf::quaternionMsgToTF(pose2d.orientation, quat);
  double yaw = tf::getYaw(quat);
  if (yaw < 0)
    yaw += 2.0 * M_PI;

  // Descretize angle
  
  static double one_angle_range = 2.0 * M_PI / theta_size_;
  *index_theta = yaw / one_angle_range;
  *index_theta %= theta_size_;
}

void AstarSearch::pointToIndex(const geometry_msgs::Point& point, int* index_x, int* index_y)
{
  geometry_msgs::Pose pose;
  pose.position = point;
  int index_theta;
  poseToIndex(pose, index_x, index_y, &index_theta);
}

bool AstarSearch::isOutOfRange(int index_x, int index_y)
{
  if (index_x < 0 || index_x >= static_cast<int>(costmap_.info.width) || index_y < 0 ||
      index_y >= static_cast<int>(costmap_.info.height))
  {
    return true;
  }
  return false;
}

// A*最核心的 搜索步骤，即openlist的一系列操作：
bool AstarSearch::search()
{
  ros::WallTime begin = ros::WallTime::now();

  // Start A* search
  // If the openlist is empty, search failed
  // openlist不为空，就还可以搜索
  while (!openlist_.empty())
  {
    // Check time and terminate if the search reaches the time limit
    ros::WallTime now = ros::WallTime::now();
    double msec = (now - begin).toSec() * 1000.0;
    // 对搜索时间进行限制，如果超过了time_limit_，就停止搜索
    if (msec > time_limit_)
    {
      // ROS_WARN("Exceed time limit of %lf [ms]", time_limit_);
      return false;
    }

    // Pop minimum cost node from openlist
    // 从openlist中选择代价最小的node，并将其从openlist中删掉
    SimpleNode top_sn = openlist_.top();  // openlist是优先队列，会按照预设的规则，将优先级最高的node放在队首（这里就是代价小 就是 优先）
    openlist_.pop();

    // Expand nodes from this node
    // 将top_sn这个权重最小的node作为当前点，并根据index，获取一下这个node保存的基本信息；
    AstarNode* current_an = &nodes_[top_sn.index_y][top_sn.index_x][top_sn.index_theta];
    // current node已经走过了，所以给它打上closed标签
    current_an->status = STATUS::CLOSED;

    // Goal check
    // 判断一下current node是否已经是Goal
    // if (isGoal(current_an->x, current_an->y, current_an->theta))
    // {
    //   // ROS_INFO("Search time: %lf [msec]", (now - begin).toSec() * 1000.0);
    //   setPath(top_sn);  // 如果当前点已经是终点，则从终点开始，倒叙回溯，得到完整的 规划路径
    //   return true;  // 如果成功搜索，就会在这里返回true，否则直接在最下面返回false，表示规划失败
    // }
    
    // 如果需要使用RS曲线
    if(use_rs_curve_){
      // 在普通A*扩展到离goal 距离 shot_distance 内，并且角度差也在M_PI_2时，就停止普通A*的扩展，而是从当前扩展到的node开始，使用RS曲线向goal进行扩展
      if (calcDistance(current_an->x,current_an->y,goal_node_.x,goal_node_.y) <= shot_distance && 
                      fabs(Mod2Pi(current_an->theta - goal_node_.theta))<= M_PI_2) {
        ROS_INFO("A* to the goal!");
        double rs_length = 0.0;
        if (AnalyticExpansions(current_an, &goal_node_, rs_length,rs_path_)) {
            setPath(top_sn);
            setRSPath(rs_path_);  // 在原A*的路径后面，把RS曲线的路径也加上
            ROS_INFO("\033[1;32m --> Astar find path! \033[0m\n");
            return true;
        }
      }
    }
    else{
      if (isGoal(current_an->x, current_an->y, current_an->theta))
      {
        // ROS_INFO("Search time: %lf [msec]", (now - begin).toSec() * 1000.0);
        setPath(top_sn);  // 如果当前点已经是终点，则从终点开始，倒叙回溯，得到完整的 规划路径
        return true;  // 如果成功搜索，就会在这里返回true，否则直接在最下面返回false，表示规划失败
      }
    }


    // Expand nodes
    // 如果还未到终点，就继续后面的搜索过程：遍历current node周围的相邻nodes
    /*
    关于如何遍历相邻nodes，这里稍微有点绕：
      提前定义好了一个数组：state_update_table_[top_sn.index_theta]，已经初始化好了往各个方向的搜索位移
      关于这个数组的设置，有一个AstarSearch::createStateUpdateTable()函数
    */
    // 提前设置好了六个方向的位移：forward、forward right、forward left、backward、backward left、backward right
    for (const auto& state : state_update_table_[top_sn.index_theta])
    {
      // Next state
      // 当前位置 + shift位移量，得到一个新的 备选位置
      double next_x = current_an->x + state.shift_x;
      double next_y = current_an->y + state.shift_y;
      double next_theta = modifyTheta(current_an->theta + state.rotation);
      double move_cost = state.step;
      double move_distance = current_an->move_distance + state.step;

      // Increase reverse cost
      // 如果轨迹有速度的正负变换，即从前进变为后退，或者从后退变为前进，还要额外惩罚
      // TODO:惩罚的参数调整
      if (state.back != current_an->back)
        move_cost *= reverse_weight_;

      // Calculate index of the next state
      SimpleNode next_sn;
      geometry_msgs::Point next_pos;
      next_pos.x = next_x;
      next_pos.y = next_y;
      pointToIndex(next_pos, &next_sn.index_x, &next_sn.index_y);  // 把新的备选位置，转换成栅格地图下的index
      next_sn.index_theta = top_sn.index_theta + state.index_theta;

      // Avoid invalid index
      next_sn.index_theta = (next_sn.index_theta + theta_size_) % theta_size_;

      // Check if the index is valid
      if (isOutOfRange(next_sn.index_x, next_sn.index_y) || detectCollision(next_sn))  // 判断是否 超过 costmap边界，或者撞到障碍物
      {
        continue;
      }

      // 更新这个 新的备选位置 的 代价值
      AstarNode* next_an = &nodes_[next_sn.index_y][next_sn.index_x][next_sn.index_theta];
      double next_gc = current_an->gc + move_cost;
      double next_hc = nodes_[next_sn.index_y][next_sn.index_x][0].hc;  // wavefront or distance transform heuristic

      // increase the cost with euclidean distance
      if (use_potential_heuristic_)
      {
        next_gc += nodes_[next_sn.index_y][next_sn.index_x][0].hc;
        next_hc += calcDistance(next_x, next_y, goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y) *
                   distance_heuristic_weight_;
      }

      // increase the cost with euclidean distance
      if (!use_wavefront_heuristic_ && !use_potential_heuristic_)
      {
        next_hc = calcDistance(next_x, next_y, goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y) *
                  distance_heuristic_weight_;
      }

      // NONE
      // 如果这个被选位置 的状态是NONE，表示之前还没有搜粟过，那么就把它加入到openlist中
      if (next_an->status == STATUS::NONE)
      {
        next_an->status = STATUS::OPEN;
        next_an->x = next_x;
        next_an->y = next_y;
        next_an->theta = next_theta;
        next_an->gc = next_gc;
        next_an->hc = next_hc;
        next_an->move_distance = move_distance;
        next_an->back = state.back;
        next_an->parent = current_an;
        next_sn.cost = next_an->gc + next_an->hc;
        openlist_.push(next_sn);
        continue;
      }

      // OPEN or CLOSED
      // 如果这个被选位置 的状态是OPEN或者CLOSED，表示之前已经搜过了，那么就比较一下，如果新的代价值更小，就更新一下信息
      if (next_an->status == STATUS::OPEN || next_an->status == STATUS::CLOSED)
      {
        if (next_gc < next_an->gc)
        {
          // 如果小于，那么就将状态变成Open，并更新这个node的最新权重。同时更重要的是更改这个node的父节点为current node
          next_an->status = STATUS::OPEN;
          next_an->x = next_x;
          next_an->y = next_y;
          next_an->theta = next_theta;
          next_an->gc = next_gc;
          next_an->hc = next_hc;  // already calculated ?
          next_an->move_distance = move_distance;
          next_an->back = state.back;
          next_an->parent = current_an;  // 更新父节点
          next_sn.cost = next_an->gc + next_an->hc;
          openlist_.push(next_sn);
          continue;
        }
      }
    }  // state update
  }

  // Failed to find path
  // ROS_INFO("Open list is empty...");
  return false;  // 如果前面的循环中 都没有成功返回，就表示搜索失败，会在这里返回false
}

// A*的最后一步，从终点开始，根据节点的父节点属性node->parent，倒叙回到起点，并在最后 颠倒vector，得到正序的规划路径
// FIXME：这里为了让astar_navi规划出的全局轨迹可以同时包含正和负的速度点，所以在A*的最后一步中，额外在一个vector中记录每个点的back属性，然后在astar_navi中就可以根据这个vector去赋值正负
void AstarSearch::setPath(const SimpleNode& goal)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = costmap_.header.frame_id;
  path_.header = header;

  // From the goal node to the start node
  // 首先把goal对应的node找到
  AstarNode* node = &nodes_[goal.index_y][goal.index_x][goal.index_theta];

  // 所有有意义的node都在openlist中，只要通过一个循环，从goal对应的node开始，不断向上查找parent node，直到node != NULL；
  while (node != NULL)
  {
    // Set tf pose
    tf::Vector3 origin(node->x, node->y, 0);
    tf::Pose tf_pose;
    tf_pose.setOrigin(origin);
    tf_pose.setRotation(tf::createQuaternionFromYaw(node->theta));

    // khy add
    // node->back为1表示 该点速度为负！
    velocity_flag_.push_back(node->back);  // 将 规划路径中 每个轨迹点的速度正负信息也存入。node是Astarnode的指针，Astarnode才是有back属性的，而非path_.poses!

    // Set path as ros message
    geometry_msgs::PoseStamped ros_pose;
    tf::poseTFToMsg(tf_pose, ros_pose.pose);
    ros_pose.header = header;
    path_.poses.push_back(ros_pose);  // path_这个属性才是astar_search最终返回的规划路径。astar的成员函数getPath()可以获取这个path_属性

    // To the next node
    node = node->parent;  // 从终点开始，根据节点的父节点属性node->parent，倒叙把每个规划点放入一个vector中
  }

  // Reverse the vector to be start to goal order
  // 由于找的过程是从goal到start一个相反的过程，所以向上搜索结束后，要有一步取反过程；
  std::reverse(path_.poses.begin(), path_.poses.end());

  // khy add:
  std::reverse(velocity_flag_.begin(), velocity_flag_.end());  // 同理，记录的速度信息也是倒叙的，所以要颠倒
  velocity_flag_[0] = velocity_flag_[1];  // 平滑前两个点的方向
}

// Check if the next state is the goal
// Check lateral offset, longitudinal offset and angle
// 判断终点的时候起始是一个阈值范围，只有达到了预设的终点附近，并且orientation也达到了预设的角度范围，就算到达了终点
bool AstarSearch::isGoal(double x, double y, double theta)
{
  // To reduce computation time, we use square value for distance
  static const double lateral_goal_range =
      lateral_goal_range_ / 2.0;  // [meter], divide by 2 means we check left and right
  static const double longitudinal_goal_range =
      longitudinal_goal_range_ / 2.0;                                         // [meter], check only behind of the goal
  static const double goal_angle = M_PI * (angle_goal_range_ / 2.0) / 180.0;  // degrees -> radian

  // Calculate the node coordinate seen from the goal point
  tf::Point p(x, y, 0);
  geometry_msgs::Point relative_node_point = calcRelativeCoordinate(goal_pose_local_.pose, p);

  // Check Pose of goal
  if (relative_node_point.x < 0 &&  // shoud be behind of goal
      std::fabs(relative_node_point.x) < longitudinal_goal_range &&
      std::fabs(relative_node_point.y) < lateral_goal_range)
  {
    // Check the orientation of goal
    if (calcDiffOfRadian(goal_yaw_, theta) < goal_angle)
    {
      return true;
    }
  }

  return false;
}

// 栅格地图nodes_已经在initialize()中更新了障碍物的节点 状态 为 STATUS::OBS
bool AstarSearch::isObs(int index_x, int index_y)
{
  if (nodes_[index_y][index_x][0].status == STATUS::OBS)
  {
    return true;
  }

  return false;
}

// 碰撞检测
bool AstarSearch::detectCollision(const SimpleNode& sn)
{
  // Define the robot as rectangle
  // 车在行驶的过程中肯定不能简单地当作质点，它也有长宽高，得到一个 占据多个栅格 的车辆表示
  static double left = -1.0 * robot_base2back_;
  static double right = robot_length_ - robot_base2back_;
  static double top = robot_width_ / 2.0;
  static double bottom = -1.0 * robot_width_ / 2.0;
  static double resolution = costmap_.info.resolution;

  // Coordinate of base_link in OccupancyGrid frame
  static double one_angle_range = 2.0 * M_PI / theta_size_;
  double base_x = sn.index_x * resolution;
  double base_y = sn.index_y * resolution;
  double base_theta = sn.index_theta * one_angle_range;

  // Calculate cos and sin in advance
  double cos_theta = std::cos(base_theta);
  double sin_theta = std::sin(base_theta);

  // Convert each point to index and check if the node is Obstacle
  // 就是判断 车辆占据栅格 是否 为 STATUS::OBS 状态，即车辆是否 走到了 有障碍物的 栅格
  for (double x = left; x < right; x += resolution)
  {
    for (double y = top; y > bottom; y -= resolution)
    {
      // 2D point rotation
      int index_x = (x * cos_theta - y * sin_theta + base_x) / resolution;
      int index_y = (x * sin_theta + y * cos_theta + base_y) / resolution;

      if (isOutOfRange(index_x, index_y))
      {
        return true;
      }
      else if (nodes_[index_y][index_x][0].status == STATUS::OBS)
      {
        return true;
      }
    }
  }

  return false;
}

bool AstarSearch::calcWaveFrontHeuristic(const SimpleNode& sn)
{
  // Set start point for wavefront search
  // This is goal for Astar search
  nodes_[sn.index_y][sn.index_x][0].hc = 0;
  WaveFrontNode wf_node(sn.index_x, sn.index_y, 1e-10);
  std::queue<WaveFrontNode> qu;
  qu.push(wf_node);

  // State update table for wavefront search
  // Nodes are expanded for each neighborhood cells (moore neighborhood)
  double resolution = costmap_.info.resolution;
  static std::vector<WaveFrontNode> updates = {
    getWaveFrontNode(0, 1, resolution),
    getWaveFrontNode(-1, 0, resolution),
    getWaveFrontNode(1, 0, resolution),
    getWaveFrontNode(0, -1, resolution),
    getWaveFrontNode(-1, 1, std::hypot(resolution, resolution)),
    getWaveFrontNode(1, 1, std::hypot(resolution, resolution)),
    getWaveFrontNode(-1, -1, std::hypot(resolution, resolution)),
    getWaveFrontNode(1, -1, std::hypot(resolution, resolution)),
  };

  // Get start index
  int start_index_x;
  int start_index_y;
  int start_index_theta;
  poseToIndex(start_pose_local_.pose, &start_index_x, &start_index_y, &start_index_theta);

  // Whether the robot can reach goal
  bool reachable = false;

  // Start wavefront search
  while (!qu.empty())
  {
    WaveFrontNode ref = qu.front();
    qu.pop();

    WaveFrontNode next;
    for (const auto& u : updates)
    {
      next.index_x = ref.index_x + u.index_x;
      next.index_y = ref.index_y + u.index_y;

      // out of range OR already visited OR obstacle node
      if (isOutOfRange(next.index_x, next.index_y) || nodes_[next.index_y][next.index_x][0].hc > 0 ||
          nodes_[next.index_y][next.index_x][0].status == STATUS::OBS)
      {
        continue;
      }

      // Take the size of robot into account
      if (detectCollisionWaveFront(next))
      {
        continue;
      }

      // Check if we can reach from start to goal
      if (next.index_x == start_index_x && next.index_y == start_index_y)
      {
        reachable = true;
      }

      // Set wavefront heuristic cost
      next.hc = ref.hc + u.hc;
      nodes_[next.index_y][next.index_x][0].hc = next.hc;

      qu.push(next);
    }
  }

  // End of search
  return reachable;
}

// Simple collidion detection for wavefront search
bool AstarSearch::detectCollisionWaveFront(const WaveFrontNode& ref)
{
  // Define the robot as square
  static double half = robot_width_ / 2;
  double robot_x = ref.index_x * costmap_.info.resolution;
  double robot_y = ref.index_y * costmap_.info.resolution;

  for (double y = half; y > -1.0 * half; y -= costmap_.info.resolution)
  {
    for (double x = -1.0 * half; x < half; x += costmap_.info.resolution)
    {
      int index_x = (robot_x + x) / costmap_.info.resolution;
      int index_y = (robot_y + y) / costmap_.info.resolution;

      if (isOutOfRange(index_x, index_y))
      {
        return true;
      }

      if (nodes_[index_y][index_x][0].status == STATUS::OBS)
      {
        return true;
      }
    }
  }

  return false;
}

// 清空openlist，重置 栅格地图nodes_的状态 为 STATUS::NONE
void AstarSearch::reset()
{
  path_.poses.clear();
  velocity_flag_.clear();

  // Clear queue
  std::priority_queue<SimpleNode, std::vector<SimpleNode>, std::greater<SimpleNode>> empty;
  std::swap(openlist_, empty);

  // ros::WallTime begin = ros::WallTime::now();

  // Reset node info here ...?
  for (size_t i = 0; i < costmap_.info.height; i++)
  {
    for (size_t j = 0; j < costmap_.info.width; j++)
    {
      for (int k = 0; k < theta_size_; k++)
      {
        // other values will be updated during the search
        nodes_[i][j][k].status = STATUS::NONE;
        nodes_[i][j][k].hc = 0;
      }
    }
  }

  // ros::WallTime end = ros::WallTime::now();

  // ROS_INFO("Reset time: %lf [ms]", (end - begin).toSec() * 1000);
}


bool AstarSearch::AnalyticExpansions(const AstarNode* current_node, const AstarNode* goal_node, double &length, 
                          VectorVec4d &rs_path_with_velocity){
                            
    Vec3d current_pose(current_node->x,current_node->y,current_node->theta);
    Vec3d goal_pose(goal_node->x,goal_node->y,goal_node->theta);
    // std::cout<<current_pose.x()<<", "<<current_pose.y()<<", "<<current_pose.z()<<std::endl;
    // std::cout<<goal_pose.x()<<", "<<goal_pose.y()<<", "<<goal_pose.z()<<std::endl;
    // std::cout<<"0000"<<std::endl;
    VectorVec3d path = rs_path_ptr_->GetRSPath(current_pose,goal_pose, 0.5, length);  // 得到从当前拓展到的节点，到终点的RS曲线

    SimpleNode next_sn;
    geometry_msgs::Point next_pos;
    static double angle_range =  2.0 * M_PI / theta_size_;

    // 检查一下RS曲线是否合法
    for(const auto &pose:path){
      next_pos.x = pose.x();
      next_pos.y = pose.y();
      pointToIndex(next_pos, &next_sn.index_x, &next_sn.index_y);  // 把新的备选位置，转换成栅格地图下的index
      // Avoid invalid index
      int ori_index = pose.z()/angle_range;
      next_sn.index_theta = ori_index%theta_size_;

      // Check if the index is valid
      if (isOutOfRange(next_sn.index_x, next_sn.index_y) || detectCollision(next_sn))  // 判断是否 超过 costmap边界，或者撞到障碍物
      {
        return false;
      }
    }

    // rs_path_with_velocity是四维向量，前两维是xy位置，第三维是yaw角，第四维是速度方向
    rs_path_with_velocity.clear();

    for (int i = 1;i<path.size();i++)
    {
      Vec4d dir_pose;
      dir_pose.head(3) = path.at(i);
      double tau = atan2(path.at(i)[1] - path.at(i-1)[1], path.at(i)[0] - path.at(i-1)[0]);
      if(fabs(Mod2Pi(path.at(i)[2]) - tau)<M_PI_2)
          dir_pose[3] = 0;
      else
          dir_pose[3] = 1;
      
      rs_path_with_velocity.push_back(dir_pose);
    }

    for(auto &pt:rs_path_with_velocity)
    {
        pt[2] = Mod2Pi(pt[2]);
    }
    return true;                           
}

double AstarSearch::Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);
    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

// 把RS曲线转到 geometry_msgs::PoseStamped，塞入原本A*轨迹所在的path_中，同时还有方向信息velocity_flag_
void AstarSearch::setRSPath(VectorVec4d &rs_path_with_velocity){
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = costmap_.header.frame_id;
  path_.header = header;

  for(auto &point:rs_path_with_velocity){
    tf::Vector3 origin(point[0], point[1], 0);
    tf::Pose tf_pose;
    tf_pose.setOrigin(origin);
    tf_pose.setRotation(tf::createQuaternionFromYaw(point[2]));

    velocity_flag_.push_back(static_cast<bool>(point[3]));

    geometry_msgs::PoseStamped ros_pose;
    tf::poseTFToMsg(tf_pose, ros_pose.pose);
    ros_pose.header = header;
    path_.poses.push_back(ros_pose); 
  }
}