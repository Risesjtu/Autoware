#include "scenario_manager/scenario_manager_core.h"

void ParkArea::push_back(const PlannerHNS::GPSPoint &point){
  vertexs_.push_back(point);
}

bool ParkArea::isInside(const PlannerHNS::GPSPoint &rough_goal){
  double current_result = 0, pre_result = 0;
  for(int i = 0;i<vertexs_.size();i++){
    Eigen::Vector3d v1(rough_goal.x - vertexs_[i].x, rough_goal.y - vertexs_[i].y,0);
    PlannerHNS::GPSPoint &next_pt = vertexs_[(i+1)%vertexs_.size()];
    Eigen::Vector3d v2(next_pt.x - vertexs_[i].x, next_pt.y - vertexs_[i].y,0);
    current_result = v1.cross(v2)[2];
    if(i>0 && current_result*pre_result<=0) 
       return false;
    pre_result = current_result;
  }
  return true;
}


Scenario::Scenario(autoware_msgs::ScenarioCmd planning_cmd){
   planning_cmd_ = planning_cmd;
}

void Scenario::set_planning_cmd(autoware_msgs::ScenarioCmd planning_cmd){
  planning_cmd_ = planning_cmd;
}

void Scenario::push_control_cmd(std::pair<string,autoware_msgs::CleaningStatus> control_cmd){
  control_cmd_que_.push(control_cmd);
}


void Scenario::pop_control_cmd(){
  if(control_cmd_que_.size() != 0){
    control_cmd_que_.pop();
  }
}

ScenarioManager::ScenarioManager(){
    scenario_ended = false;
    start_pose_initialized_ = false;

    is_goal_empty_.data = 1;  // 终点队列是否为空

    tf::StampedTransform transform;
    PlannerHNS::ROSHelpers::GetTransformFromTF("map", "world", transform);  // 获取world到map的变换
    // origin_pose_在后续用来进行坐标系变换
    origin_pose_.position.x  = transform.getOrigin().x();
    origin_pose_.position.y  = transform.getOrigin().y();
    origin_pose_.position.z  = transform.getOrigin().z();

    sub_points_ = nh_.subscribe("/vector_map_info/point", 1, &ScenarioManager::callbackGetVMPoints,  this);
    sub_lines_ = nh_.subscribe("/vector_map_info/line", 1, &ScenarioManager::callbackGetVMLines,  this);
    sub_area_ = nh_.subscribe("/vector_map_info/area", 1, &ScenarioManager::callbackGetVMAreas,  this);
    sub_park_area_ = nh_.subscribe("/vector_map_info/custom_area", 1, &ScenarioManager::callbackGetVMCustomAreas,this);
    sub_lanes_ = nh_.subscribe("/vector_map_info/lane", 1, &ScenarioManager::callbackGetVMLanes,this);
    sub_nodes_ = nh_.subscribe("/vector_map_info/node", 1, &ScenarioManager::callbackGetVMNodes,this);
    sub_goal_pose_ = nh_.subscribe("move_base_simple/goal", 1, &ScenarioManager::callbackGetRvizGoal, this);
    sub_current_pose_ = nh_.subscribe("/current_pose", 1, &ScenarioManager::callbackGetCurrentPose, this);

    // 订阅 从录制轨迹起点 计算得来的终点
    sub_goal_from_loading_ = nh_.subscribe("/based/goal_from_loading", 1, &ScenarioManager::callbackGetGoalFromLoading, this);

    // 订阅每个场景结束的标志位
    sub_scenario_end_status = nh_.subscribe("/scenario_ended",1,&ScenarioManager::callbackGetScenarioEnded,this);

    // 订阅 执行的控制命令，是否有 已结束的反馈
    sub_cmd_feedback = nh_.subscribe("/control_feedback", 1, &ScenarioManager::callbackGetControlFeedback, this);
    // 发布当前所要运转的场景
    pub_scenario_ = nh_.advertise<autoware_msgs::ScenarioCmd>("scenario_manager/scenario_cmd",1);
    pub_cleaning_cmd_ = nh_.advertise<autoware_msgs::CleaningStatus>("cleaning_status",1);
    pub_IsGoalEmpty = nh_.advertise<std_msgs::Int32>("is_goal_empty", 1);  // for app
    rvizMarkerPub_ = nh_.advertise < visualization_msgs::Marker > ("goal_marker", 1);
     
    // 创建时间回调，以0.2S的频率去执行内部的操作，主要是时刻准备接收每个场景的结束标志位，然后pop掉当前的，就自然进入到下一个场景
    timer = nh_.createTimer(ros::Duration(0.2), &ScenarioManager::callbackScenarioPub,this);
    nh_.param("/scenario_manager/max_dist_to_lane_driving_goal", max_dist_to_lane_driving_goal, 6.0);
    nh_.param("/scenario_manager/min_dist_to_lane_driving_goal", min_dist_to_lane_driving_goal, 6.0);
    nh_.param("/scenario_manager/enableCover", enableCover, true);

    ROS_INFO("max_dist_to_lane_driving_goal is set %f", max_dist_to_lane_driving_goal);
    ROS_INFO("min_dist_to_lane_driving_goal is set %f", min_dist_to_lane_driving_goal);


    int bVelSource = 1;
    nh_.getParam("/scenario_manager/velocitySource", bVelSource);
    if(bVelSource == 0)
      sub_robot_odom_ = nh_.subscribe("/odom", 10, &ScenarioManager::callbackGetRobotOdom, this);
    else if(bVelSource == 1)
      sub_current_velocity_ = nh_.subscribe("/current_velocity", 10, &ScenarioManager::callbackGetVehicleStatus, this);
    else if(bVelSource == 2)
      sub_can_info_ = nh_.subscribe("/can_info", 10, &ScenarioManager::callbackGetCANInfo, this);
}

ScenarioManager::~ScenarioManager(){  

}


void ScenarioManager::create_park_area_vmap(){
    // 需要确定 矢量信息中的 点，线，区域，自定义区域 都订阅到
    if(all_vmap_.points.empty() || all_vmap_.lines.empty()||all_vmap_.areas.empty() || all_vmap_.custom_areas.empty())
       return;
    for(auto &custom_area:all_vmap_.custom_areas){
      ParkArea park_area;
      int aid = custom_area.aid;
      auto area = all_vmap_.areas[aid-1];

      // get the start line id and end line id
      int start_line_id = area.slid, end_line_id = area.elid;

      // get the start point id and end point id
      int start_point_id = all_vmap_.lines[start_line_id-1].bpid, end_point_id = all_vmap_.lines[end_line_id-1].bpid;

      for(int i = start_point_id - 1;i<=end_point_id-1;i++){
        auto point = all_vmap_.points[i];
        PlannerHNS::GPSPoint vertex(point.ly,point.bx,0,0);  // 构造一个GPSPoint
        park_area.push_back(vertex);
      }
      park_areas_.emplace_back(park_area);
    }
}

bool ScenarioManager::find_nearest_goal_in_lane(const PlannerHNS::GPSPoint &rough_goal, geometry_msgs::PoseStamped &modified_goal,
                                                int flag)
{
    if(all_vmap_.lanes.empty() || all_vmap_.points.empty() || all_vmap_.nodes.empty()) return false;
    // 遍历所有的lane
    for(const auto &lane:all_vmap_.lanes){
      int node_id = lane.fnid;
      int point_id = all_vmap_.nodes[node_id - 1].pid;
      // 遍历每条lane上的所有点，point
      PlannerHNS::GPSPoint point(all_vmap_.points[point_id - 1].ly,all_vmap_.points[point_id - 1].bx,0,0);
      if(fabs(point.x - rough_goal.x) > max_dist_to_lane_driving_goal || fabs(point.y - rough_goal.y)>max_dist_to_lane_driving_goal)
      {
           continue;
      }
      double distance = calculate_distance(point,rough_goal);  // 计算point和第一个参数rough_goal的距离
      ROS_INFO("The satisfied point, x: %f, y: %f, distance :%f",point.x,point.y,distance);
      if(distance <=max_dist_to_lane_driving_goal && distance >= min_dist_to_lane_driving_goal){ 

        int pre_node_id = lane.bnid;
        int pre_point_id = all_vmap_.nodes[pre_node_id - 1].pid;
        PlannerHNS::GPSPoint pre_point(all_vmap_.points[pre_point_id - 1].ly,all_vmap_.points[pre_point_id - 1].bx,0,0);
        double yaw = atan2(point.y - pre_point.y, point.x - pre_point.x);

        // 准备goal的向量，和遍历的lane的向量（rough_goal_vec是停车位向外的方向向量，称为纵向，lane_vec是横向lane上的点减去rough_goal这段向量）
        Eigen::Vector3d rough_goal_vec(cos(rough_goal.a),sin(rough_goal.a),0);
        Eigen::Vector3d lane_vec(point.x - rough_goal.x,point.y - rough_goal.y, 0);

        // flag is 0 to find stop goal, the goal is on rough_goal left
        // flag is 1 to find start goal, the goal is on rough_goal right
        double dir_judge = rough_goal_vec.cross(lane_vec)[2];  // 叉乘，判断方向
        // 如果flag是0，并且 lane 在 rough_goal 的左边（即rough_goal_vec纵向，lane_vec在rough_goal_vec的左边，是横向的）；
        // 或者flag是1，并且 lane 在 rough_goal 的右边
        // 通过右手螺旋判断出两种情况，只有有某条lane满足了方向（和上面的距离），就返回true，并且把这个lane的点赋值给modified_goal
        if((flag == 0 && dir_judge > 0) || (flag == 1 && dir_judge < 0)){     
          // 会对modified_goal进行赋值，由于是引用方式传参，所以会直接修改原来的modified_goal   
          modified_goal.pose.position.x = point.x; // map 坐标系下
          modified_goal.pose.position.y = point.y;
          modified_goal.pose.position.z += origin_pose_.position.z;
          
          auto quaternion = tf::createQuaternionFromYaw(yaw);
          modified_goal.pose.orientation.x = quaternion.x();
          modified_goal.pose.orientation.y = quaternion.y();
          modified_goal.pose.orientation.z = quaternion.z();
          modified_goal.pose.orientation.w = quaternion.w();
          return true;
        }
      }
    }
    return false;
}



// for vector_map info
void ScenarioManager::callbackGetVMPoints(const vector_map::PointArray& msg)
{
	std::cout << "Received Points" << msg.data.size() << std::endl;
    if(!msg.data.empty())
    {
        all_vmap_.points = msg.data;
        create_park_area_vmap();
    }
}

void ScenarioManager::callbackGetVMLines(const vector_map::LineArray& msg)
{
	std::cout << "Received Lines" << msg.data.size() << std::endl;
    if(!msg.data.empty())
    {
       all_vmap_.lines = msg.data;
       create_park_area_vmap();
    }
}

void ScenarioManager::callbackGetVMAreas(const vector_map::AreaArray& msg)
{
	std::cout << "Received areas" << msg.data.size() << std::endl;
    if(!msg.data.empty())
    {
        all_vmap_.areas= msg.data;
        create_park_area_vmap();
    }
}

void ScenarioManager::callbackGetVMCustomAreas(const vector_map::CustomAreaArray& msg)
{
  	std::cout << "Received customArea" << msg.data.size() << std::endl;
    if(!msg.data.empty())
    {
        all_vmap_.custom_areas = msg.data;
        create_park_area_vmap();
    }
}

void ScenarioManager::callbackGetVMLanes(const vector_map::LaneArray &msg){
  std::cout<< "Received Lanes"<<msg.data.size()<<std::endl;
  if(!msg.data.empty()){
    all_vmap_.lanes = msg.data;
  }

}

void ScenarioManager::callbackGetVMNodes(const vector_map::NodeArray &msg){
  std::cout<< "Received Nodes"<<msg.data.size()<<std::endl;
  if(!msg.data.empty()){
    all_vmap_.nodes = msg.data;
  }
}

void ScenarioManager::callbackGetScenarioEnded(const std_msgs::Bool &msg){
   scenario_ended = msg.data;
   ROS_INFO("Get the end status!");
}

void ScenarioManager::callbackGetGoalFromLoading(const geometry_msgs::PoseStamped& msg){
  goal_from_loading_ = msg;
}

// get rviz initial pose
// 场景的初始化都在这个回调中完成，包括后续如果还要增加场景状态也应该在下面的回调中设计逻辑
// 现阶段设计了 三个场景状态：从停车区域开出去 开到op lane -> 在op lane上行驶 -> 从op lane 往 停车区域内停车
void ScenarioManager::callbackGetRvizGoal(const autoware_msgs::DockInfoConstPtr &msg){
    autoware_msgs::DockInfo new_goal = *msg;
    goal_que_.push(new_goal);
}


void ScenarioManager::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg){
    current_pose_.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));

}


// 处理 当前场景结束后，事先与其绑定的控制队列 中的控制命令
void ScenarioManager::deal_control_cmd(Scenario &current_scenario){
  auto &control_que = current_scenario.control_cmd_que_;
  ROS_INFO_STREAM("current control que size: " << control_que.size());
  // 如果控制队列为空，表示当前场景结束后没有要执行的控制命令，直接返回
  if(control_que.size()==0) {
     ROS_INFO_STREAM("control operations after the current scenario have all ended");
     return;
  }
  
  auto &control_pair = control_que.front();  // 取出当前第一个控制命令
  ROS_INFO_STREAM("current control operation: " << control_pair.first);
  // 将控制命令发布出去，并设置一个标志位，避免一条命令重复发布
  if(!pre_control_published){
    pub_cleaning_cmd_.publish(control_pair.second);
    pre_control_published = true;
    return;
  }
  // 只有当前控制命令有返回一个 结束的标志，才会进入到下面的if，把做完的控制命令pop掉，并发布控制队列中的下一个控制命令
  ROS_INFO_STREAM("current control feedback: " << control_cmd_feedback_[control_pair.first]);
  if(control_cmd_feedback_[control_pair.first]){
    // control_cmd_feedback_是在另一个订阅回调中不断更新的
    control_cmd_feedback_[control_pair.first] = false;  // control_cmd_feedback_这个map容器中存储的是 所有 控制操作 的反馈状态
    current_scenario.pop_control_cmd();
    if(control_que.size()>0){
      pub_cleaning_cmd_.publish(control_pair.second);
      ROS_INFO_STREAM("switch to next control order!");
      pre_control_published = true;
    }
  }
}

void ScenarioManager::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
  current_pose_.v = msg->twist.twist.linear.x;
}

void ScenarioManager::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
  current_pose_.v = msg->twist.linear.x;
}

void ScenarioManager::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
  current_pose_.v = msg->speed/3.6;
}

void ScenarioManager::callbackGetControlFeedback(const autoware_msgs::VehicleCleaningFeedbackConstPtr &msg){
  // control_cmd_feedback_["forward"] = msg->forward.data;
  // control_cmd_feedback_["backward"] = msg->backward.data;
  // control_cmd_feedback_["stop"] = msg->stop.data;
  control_cmd_feedback_["cleaning"] = msg->cleaning.data;
  control_cmd_feedback_["garbaging"] = msg->garbaging.data;  // 倒垃圾控制命令的 反馈消息，表示是否完成倒垃圾
  control_cmd_feedback_["watering"] = msg->watering.data;
}

// 把ScenarioCmd中的goal成员，转换到world坐标系
autoware_msgs::ScenarioCmd  ScenarioManager::convert_to_world_pose(const autoware_msgs::ScenarioCmd &scenario){
  autoware_msgs::ScenarioCmd world_pose = scenario;
  world_pose.goal.pose.position.x -= origin_pose_.position.x;
  world_pose.goal.pose.position.y -= origin_pose_.position.y;
  world_pose.goal.pose.position.z -= origin_pose_.position.z;
  return world_pose;
}

void ScenarioManager::generate_scenario(autoware_msgs::DockInfo &current_goal, bool cover_flag) {
  pre_scenario_published = false;

  geometry_msgs::PoseStamped goal_pose = current_goal.goal;
  // Rviz勾选的点是在world坐标系，先加上origin_pose_，变换到map坐标系下
  PlannerHNS::GPSPoint wp = PlannerHNS::GPSPoint(goal_pose.pose.position.x + origin_pose_.position.x, goal_pose.pose.position.y + origin_pose_.position.y, goal_pose.pose.position.z + origin_pose_.position.z, tf::getYaw(goal_pose.pose.orientation));
  if (park_areas_.empty()) {
    ROS_WARN("current pose is not initialized or park_area is empty!");
    return ;
  }
  
  // for start process. If car is inside park area, go to nearest lane point in parking scenario.
  // 判断起点是否在停车区域内，如果在的话，就添加一个go_to_lane_scenario的场景，并且.scenario属性要设置成1，表示使用Astar，从停车区域先走到op的正常车道上
  // 第一个场景：暂时是在停车区外 的右前侧的 全局矢量lane上找一个最近终点，使车子先走出停车区域
  bool start_pose_inside_flag = false;
  for (auto &park_area:park_areas_) {
    start_pose_inside_flag = park_area.isInside(current_pose_.pos);  // 判断当前车辆位姿是否在停车区域
    ROS_INFO_STREAM("\033[1;33m start pose inside park area: \033[0m" << start_pose_inside_flag);  // 黄色
    // ROS_INFO("start pose inside park area: %d", start_pose_inside_flag);
    if (start_pose_inside_flag) {
      geometry_msgs::PoseStamped start_goal = goal_pose;
      // 第三个参数1表示 在current_pose_.pos的右侧lane上找最近的终点，并且赋值给start_goal
      if (find_nearest_goal_in_lane(current_pose_.pos, start_goal, 1)) {  // current_pose_.pos和start_goal(=*msg)都是在world坐标系下的
          Scenario go_to_lane;  // 场景
          autoware_msgs::ScenarioCmd go_to_lane_planning;  // 场景命令
          go_to_lane_planning.scenario = 1; // parking scenario
          go_to_lane_planning.mode = 1; // exlusive，用于切换场景时，关闭前一个场景
          go_to_lane_planning.goal = start_goal; // goal pose

          std::string test = "start";
          // VisualizeTF(start_goal, test);

          visualizeMarker(start_goal, test);

          autoware_msgs::CleaningStatus cleaning_cmd_start; // control order
          cleaning_cmd_start.is_cleaning.data = true;  // 这个控制命令是 启动 扫盘

          // 写成pair类型的控制指令
          std::pair<string,autoware_msgs::CleaningStatus> cleaning_pair_start("cleaning", cleaning_cmd_start);
          go_to_lane.push_control_cmd(cleaning_pair_start);  // 放入控制队列

          // Scenario的set_planning_cmd()方法，将go_to_lane_planning这个场景命令和go_to_lane这个场景绑定（只不过第一个场景结束后没有要执行的控制操作，所以go_to_lane这个场景暂时没有其他处理）
          go_to_lane.set_planning_cmd(go_to_lane_planning);
          scenario_que_.push(go_to_lane);
          break;  // 第一次找到满足要求的goal，并设置好场景后，就break掉当前场景设置的循环
      }
    }
  }

  // 第二、三个场景，在launch里设置enableCover参数，如果是true，就加入第二、三个场景，如果是false，就跳过这两个为全覆盖相关的场景
  if (enableCover && cover_flag) {  // cover_flag 用于控制 某个终点 的场景队列是否要包含全覆盖
      // 第二个场景：全覆盖前的op
      Scenario lane_driving_before_cover;
      autoware_msgs::ScenarioCmd lane_driving_before_cover_planning;
      lane_driving_before_cover_planning.scenario = 0; // lane driving scenario
      lane_driving_before_cover_planning.mode = 1; // exlusive
      lane_driving_before_cover_planning.goal = goal_from_loading_; // 录制轨迹的起点，改为从loader节点中发布过来

      std::string test = "cover_start";
      visualizeMarker(goal_from_loading_, test);

      lane_driving_before_cover.set_planning_cmd(lane_driving_before_cover_planning);
      scenario_que_.push(lane_driving_before_cover);

      // 第三个场景：全覆盖
      geometry_msgs::PoseStamped after_cover_end_pose;

      Scenario cover;
      autoware_msgs::ScenarioCmd cover_planning;
      cover_planning.scenario = 2; // 这个2应该控制waypoint_lodaer这个节点生效。也就是1控制astar_navi生效。并且1和2可以同时让astar_avoid、velocity_set、lane_rule、lane_select生效
      cover_planning.mode = 1; // exlusive
      cover_planning.goal = after_cover_end_pose; // 录制轨迹的终点。但是由于这里是直接载入录制轨迹，所以这个终点是不需要实际有值的
      // cover_planning.cover_idx = 0;  // 表示第一段录制轨迹
      cover.set_planning_cmd(cover_planning);
      scenario_que_.push(cover);
  }


  // 第四个场景：全覆盖结束后的op
  // 这段op的终点，要根据 垃圾桶的位置（第一个终点）来寻找，如果位置在垃圾桶的custom_area区域内，那就在停车区域外的左侧，最近lane上找一个终点，使得车辆能开到停车位附近
  bool end_pose_inside_flag = false;
  for (auto &park_area : park_areas_) {
    end_pose_inside_flag = park_area.isInside(wp);
    ROS_INFO_STREAM("\033[1;33m End pose inside park area: \033[0m" << end_pose_inside_flag);  // 黄色
    if (end_pose_inside_flag) {
      geometry_msgs::PoseStamped modified_goal = goal_pose;
      // 第三个参数0表示 在wp的左侧lane上找最近的终点，并且赋值给modified_goal
      // 为什么是左侧：因为触发这里第二个场景初始化的操作是把 终点 勾选在停车区域，想要实现的场景是车辆向右开出停车区域，然后逆时针绕地图一圈，最后回到停车区域内停车
      // 第二个场景就是“逆时针绕场地一圈”，这个的结束点就在停车区域的左前外侧，（后续再从这个左前外侧开到停车区域）
      if (find_nearest_goal_in_lane(wp, modified_goal, 0)) {
          Scenario lane_driving_after_cover;
          autoware_msgs::ScenarioCmd lane_driving_after_cover_planning;
          lane_driving_after_cover_planning.scenario = 0; // lane driving scenario
          lane_driving_after_cover_planning.mode = 1; // exlusive
          lane_driving_after_cover_planning.goal = modified_goal;
          
          std::string test = "end";
          // VisualizeTF(modified_goal, test);

          visualizeMarker(modified_goal, test);

          autoware_msgs::CleaningStatus cleaning_cmd_shutdown; // control order
          cleaning_cmd_shutdown.is_cleaning.data = false;  // 这个控制命令是 关闭 扫盘

          // 写成pair类型的控制指令
          std::pair<string,autoware_msgs::CleaningStatus> cleaning_pair_shutdown("cleaning", cleaning_cmd_shutdown);
          lane_driving_after_cover.push_control_cmd(cleaning_pair_shutdown);  // 放入控制队列

          lane_driving_after_cover.set_planning_cmd(lane_driving_after_cover_planning);
          scenario_que_.push(lane_driving_after_cover);
          break;
      }
    }
  }
  
  // 第五个场景
  Scenario parking;
  geometry_msgs::PoseStamped map_pose = goal_pose;
  map_pose.pose.position.x += origin_pose_.position.x;  // map frame
  map_pose.pose.position.y += origin_pose_.position.y;
  map_pose.pose.position.z += origin_pose_.position.z;

  // 第五个场景命令，初始化时 是 设置成.scenario = 0，表示使用openPlanner，沿着op的全局矢量lane 行驶（因为如果前两个场景都没有触发的话，那就是一个正常的op planner循迹）
  autoware_msgs::ScenarioCmd parking_planning;
  parking_planning.scenario = 0; // initial is lane driving
  parking_planning.mode = 1; // exlusive
  parking_planning.goal = map_pose; // goal pose
  
  // 如果在第二个场景的终点判定时，判断出终点在停车区域，就把第当前第三个场景的.scenario属性设置成1，表示使用规划A*，从op lane上倒车到垃圾桶
  if (end_pose_inside_flag) {
    parking_planning.scenario = 1; // parking scenario
    // 新建一个控制命令：
    if (current_goal.task == "garbage") {
      autoware_msgs::CleaningStatus control_cmd; // control order
      control_cmd.is_garbaging.data = true;  // 这个控制命令就是倒垃圾
      // 虽然这里是为了倒垃圾，但是如果不给它的vehicle_status字段赋值2，会默认是0，这是一个问题
      // control_cmd.vehicle_status.data = 2;  // 必须把车辆档位的字段，也置一个值，不然默认是0

      // 写成pair类型的控制指令
      std::pair<string,autoware_msgs::CleaningStatus> control_pair("garbaging", control_cmd);
      parking.push_control_cmd(control_pair);  // 放入控制队列
    }
  }

  parking.set_planning_cmd(parking_planning);  // 第五个场景parking有了包含倒垃圾的控制队列，将parking和场景命令parking_planning绑定
  scenario_que_.push(parking);

  ROS_INFO_STREAM("\033[1;32m The current scenario queue size: \033[0m" << scenario_que_.size());  // 绿色
  // ROS_INFO_STREAM("The current scenario queue size: " << scenario_que_.size());

  // 现在只有第三个parking场景（如果终点在停车框要停车），会有结束后的控制要做（倒垃圾）
  // ROS_INFO_STREAM("after parking scenario, control que size: " << parking.control_cmd_que_.size());
  // std::cout<<"The current scenario queue size: "<<scenario_que_.size()<<std::endl;
}

// The front scenario is current scenario.
// 0.2s频率的时间回调函数，这里是实际控制 场景 切换的！
void ScenarioManager::callbackScenarioPub(const ros::TimerEvent& e){

  // pub for protection of "pub_goal" in app
  // 只有当goal_que_为空的时候，app中负责pub_goal的按钮才会生效
  if (goal_que_.size() == 0) {
    is_goal_empty_.data = 1;
  }
  else {
    is_goal_empty_.data = 0;
  }
  pub_IsGoalEmpty.publish(is_goal_empty_);

  // 先后处理两个终点的场景：将第一个终点带来的场景队列处理完毕后，再发布出第二个终点的场景队列
  if(scenario_que_.size() == 0 && goal_que_.size()>0){
      autoware_msgs::DockInfo current_goal = goal_que_.front();
      goal_que_.pop();
      if(current_goal.task == "garbage"){
        generate_scenario(current_goal, true); // true
      }
      else{
        generate_scenario(current_goal, false);
      }
  }

  if(scenario_que_.size() > 0){
    Scenario aprroval_scenario = scenario_que_.front();
    // 如果还没发布过场景命令，就把当前第一个场景命令发布出去；并设置一个标志位，避免一条场景命令重复发布
    if(!pre_scenario_published){
      pub_scenario_.publish(convert_to_world_pose(aprroval_scenario.planning_cmd_));  // 将当前场景队列的第一个场景 发布出去，作为当前运转的场景
      pre_scenario_published = true;  // 并且设置一个已经发布了场景命令的标志位
      return ;
    }
    // ROS_INFO_STREAM("scenario_ended: " << scenario_ended);
    // 如果收到了场景结束的标志位，先去处理控制队列，等控制队列都执行完毕，就把当前第一个已经执行过的场景命令pop掉，然后把下一个场景命令发布出去
    if(scenario_ended)  // scenario_ended在另一个回调中实时更新
    {
      ROS_INFO_STREAM("Processing control queue!");
      deal_control_cmd(aprroval_scenario);  // 处理当前场景结束后，事先绑定的控制队列
      // 由于这是在时间回调函数中，deal_control_cmd()会一直执行，直到在内部把控制队列都执行完毕，即control_cmd_que_.size() == 0，就可以进入到下面的if，去发布下一个场景命令
      if(aprroval_scenario.control_cmd_que_.size() == 0){   
        scenario_que_.pop();
        if(scenario_que_.size() > 0){
          pub_scenario_.publish(convert_to_world_pose(scenario_que_.front().planning_cmd_));
          ROS_INFO_STREAM("switch to next scenario! Remaining scenario (including executing) queue size: " << scenario_que_.size());
          pre_scenario_published = true;
        }
        if (scenario_que_.size() == 0) {
          ROS_INFO_STREAM("current scenario queue size is empty! Waiting for new scenario...");
        }          
        scenario_ended = false;
        pre_control_published = false;
      }
    }
  }
}

void ScenarioManager::VisualizeTF(geometry_msgs::PoseStamped pose, std::string task) 
{
  tf::Transform t; 
  tf::poseMsgToTF(pose.pose, t);
  tf::StampedTransform camToMarker(t, pose.header.stamp, pose.header.frame_id, task.c_str());
  tf_broadcaster->sendTransform(camToMarker);

}

void ScenarioManager::visualizeMarker(geometry_msgs::PoseStamped pose, std::string task)
{
  rvizMarker_.header.frame_id = "map";
  rvizMarker_.header.stamp = pose.header.stamp;

  if (task == "start") {
    rvizMarker_.id = 1;
    // 紫色
    rvizMarker_.color.r = 0.5f;
    rvizMarker_.color.g = 0.0f;
    rvizMarker_.color.b = 0.5f;
    rvizMarker_.color.a = 1.0;
  }
  if (task == "end"){
    rvizMarker_.id = 2;
    rvizMarker_.color.r = 0.5f;
    rvizMarker_.color.g = 0.0f;
    rvizMarker_.color.b = 0.0f;
    rvizMarker_.color.a = 1.0;
  }

  rvizMarker_.pose = pose.pose;

  rvizMarker_.scale.x = 1.5;
  rvizMarker_.scale.y = 0.5;
  rvizMarker_.scale.z = 0.5;
  rvizMarker_.ns = "basic_shapes";
  rvizMarker_.type = visualization_msgs::Marker::ARROW;
  rvizMarker_.action = visualization_msgs::Marker::ADD;


  rvizMarker_.lifetime = ros::Duration (100.0);
  rvizMarkerPub_.publish (rvizMarker_);
}

void ScenarioManager::MainLoop(){
  ros::Rate loop_rate(20);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }

}
