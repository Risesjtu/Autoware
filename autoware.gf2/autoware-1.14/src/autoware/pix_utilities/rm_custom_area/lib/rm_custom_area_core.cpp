#include "rm_custom_area/rm_custom_area_core.h"

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


KillStopLine::KillStopLine(){

    // nh_.param("/scenario_manager/max_dist_to_lane_driving_goal", max_dist_to_lane_driving_goal, 6.0);
    // nh_.param("/scenario_manager/min_dist_to_lane_driving_goal", min_dist_to_lane_driving_goal, 6.0);
    // nh_.param("/scenario_manager/enableCover", enableCover, true);

    // tf::StampedTransform transform;
    // PlannerHNS::ROSHelpers::GetTransformFromTF("map", "world", transform);  // 获取world到map的变换

    sub_origin_pc_ = nh_.subscribe("/points_no_ground", 1, &KillStopLine::callbackOriginPC,this);
    
    sub_scenario_cmd_ = nh_.subscribe("scenario_manager/scenario_cmd",1,&KillStopLine::callbackGetScenario, this);
    sub_current_pose_ = nh_.subscribe("/current_pose", 1, &KillStopLine::callbackGetCurrentPose, this);

    // 跟vectormap有关的
    sub_points_ = nh_.subscribe("/vector_map_info/point", 1, &KillStopLine::callbackGetVMPoints,  this);
    sub_lines_ = nh_.subscribe("/vector_map_info/line", 1, &KillStopLine::callbackGetVMLines,  this);
    sub_area_ = nh_.subscribe("/vector_map_info/area", 1, &KillStopLine::callbackGetVMAreas,  this);
    sub_custom_area_ = nh_.subscribe("/vector_map_info/custom_area", 1, &KillStopLine::callbackGetVMCustomAreas,this);
    sub_lanes_ = nh_.subscribe("/vector_map_info/lane", 1, &KillStopLine::callbackGetVMLanes,this);
    sub_nodes_ = nh_.subscribe("/vector_map_info/node", 1, &KillStopLine::callbackGetVMNodes,this);

    // 发布去除后的点云
    pub_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_no_stop_line",1);

}

KillStopLine::~KillStopLine(){  

}

void KillStopLine::callbackOriginPC(const sensor_msgs::PointCloud2::ConstPtr msg){
  if (!scenario_stop_line_)
  {
    pub_pointcloud_.publish(*msg);
  }
  else 
  {
    map_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *map_cloud_);  // 转为pcl格式

    PlannerHNS::ROSHelpers::GetTransformFromTF("map", "velodyne", transform_);  // 获取velodne到map的变换


    tf::Vector3 translation = transform_.getOrigin();
    tf::Quaternion rotation = transform_.getRotation();
    Eigen::Matrix4d transformMatrix = Eigen::Matrix4d::Identity();
    
    transformMatrix.block<3, 3>(0, 0) = Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z()).toRotationMatrix();
    transformMatrix.block<3, 1>(0, 3) << translation.x(), translation.y(), translation.z();

    Eigen::Matrix4f f = transformMatrix.cast <float> (); 

    // std::cout << "f: " << f << "\n";

    pcl::PointCloud<pcl::PointXYZI>::Ptr transform_pc;
    transform_pc = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*map_cloud_, *transform_pc, f); 
    
    for (auto park_area:park_areas_)
    {
      double min_x(100000000), min_y(100000000), max_x(-100000000), max_y(-100000000);
      for (auto gps_point : park_area.vertexs_)
      {
        if (gps_point.y < min_y)
          min_y = gps_point.y;
        if (gps_point.y > max_y)
          max_y = gps_point.y;
        if (gps_point.x < min_x)
          min_x = gps_point.x;
        if (gps_point.x > max_x)
          max_x = gps_point.x;
      }
      // std::cout << max_x << " " << max_y << " " << min_x << " " << min_y << "\n";
      RemoveCustomArea(transform_pc, max_x, max_y, min_x, min_y, transform_pc, true);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr final_pc;
    final_pc = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*transform_pc, *final_pc, f.inverse()); 

    sensor_msgs::PointCloud2 final_msg;
    pcl::toROSMsg(*final_pc, final_msg);
    final_msg.header.frame_id = "velodyne";
    pub_pointcloud_.publish(final_msg);
  }


}

// ljc add: 适用于方型车辆的点云滤除
void KillStopLine::RemoveCustomArea(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, double max_x, double max_y, double min_x, double min_y,
                                       pcl::PointCloud<pcl::PointXYZI>::Ptr out_filtered_cloud_ptr, bool remove_indices)
{
  pcl::ExtractIndices<pcl::PointXYZI> extractor;
  extractor.setInputCloud(in_cloud_ptr);
  pcl::PointIndices indices;

#pragma omp for
  for (size_t i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    if (in_cloud_ptr->points[i].x < max_x && in_cloud_ptr->points[i].x > min_x
          && in_cloud_ptr->points[i].y < max_y && in_cloud_ptr->points[i].y > min_y)
    {
      indices.indices.push_back(i);
    }
  }
  extractor.setIndices(boost::make_shared<pcl::PointIndices>(indices));
  extractor.setNegative(remove_indices);  // true removes the indices, false leaves only the indices
  extractor.filter(*out_filtered_cloud_ptr);

}

void KillStopLine::create_park_area_vmap(){
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

// for vector_map info
void KillStopLine::callbackGetVMPoints(const vector_map::PointArray& msg)
{
	std::cout << "Received Points" << msg.data.size() << std::endl;
    if(!msg.data.empty())
    {
        all_vmap_.points = msg.data;
        create_park_area_vmap();
    }
}

void KillStopLine::callbackGetVMLines(const vector_map::LineArray& msg)
{
	std::cout << "Received Lines" << msg.data.size() << std::endl;
    if(!msg.data.empty())
    {
       all_vmap_.lines = msg.data;
       create_park_area_vmap();
    }
}

void KillStopLine::callbackGetVMAreas(const vector_map::AreaArray& msg)
{
	std::cout << "Received areas" << msg.data.size() << std::endl;
    if(!msg.data.empty())
    {
        all_vmap_.areas= msg.data;
        create_park_area_vmap();
    }
}

void KillStopLine::callbackGetVMCustomAreas(const vector_map::CustomAreaArray& msg)
{
  	std::cout << "Received customArea" << msg.data.size() << std::endl;
    if(!msg.data.empty())
    {
        all_vmap_.custom_areas = msg.data;
        create_park_area_vmap();
    }
}

void KillStopLine::callbackGetVMLanes(const vector_map::LaneArray &msg){
  std::cout<< "Received Lanes"<<msg.data.size()<<std::endl;
  if(!msg.data.empty()){
    all_vmap_.lanes = msg.data;
  }

}

void KillStopLine::callbackGetVMNodes(const vector_map::NodeArray &msg){
  std::cout<< "Received Nodes"<<msg.data.size()<<std::endl;
  if(!msg.data.empty()){
    all_vmap_.nodes = msg.data;
  }
}

void KillStopLine::callbackGetScenario(const autoware_msgs::ScenarioCmd &msg){
  if (msg.scenario == 1)
  { 
    // 当1时, 为倒车(充电或倒垃圾), 此时才开始rm custom_area中的点云
    scenario_stop_line_ = true;
  }
  else 
  {
    scenario_stop_line_ = false;
  }
  ROS_INFO("begin rm custom_area point");
}

void KillStopLine::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg){
    current_pose_.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));

}

void KillStopLine::MainLoop(){
  ros::Rate loop_rate(20);
  while(ros::ok()){
    ros::spinOnce();
  loop_rate.sleep();
}

}
