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

#include "op_global_planner_core.h"
#include "op_ros_helpers/op_ROSHelpers.h"
#include "rs.h"
#include "matplotlibcpp.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>

namespace plt=matplotlibcpp;

namespace GlobalPlanningNS
{

GlobalPlanner::GlobalPlanner()
{
  m_pCurrGoal = 0;
  m_iCurrentGoalIndex = 0;
  m_bKmlMap = false;
  m_bFirstStart = false;
  m_GlobalPathID = 1;
  activate_scenario = false;
  UtilityHNS::UtilityH::GetTickCount(m_ReplnningTimer);

  nh.getParam("/op_global_planner/pathDensity" , m_params.pathDensity);
  nh.getParam("/op_global_planner/enableSmoothing" , m_params.bEnableSmoothing);
  nh.getParam("/op_global_planner/enableLaneChange" , m_params.bEnableLaneChange);
  nh.getParam("/op_global_planner/enableRvizInput" , m_params.bEnableRvizInput);
  nh.getParam("/op_global_planner/enableReplan" , m_params.bEnableReplanning);
  nh.getParam("/op_global_planner/enableDynamicMapUpdate" , m_params.bEnableDynamicMapUpdate);
  nh.getParam("/op_global_planner/mapFileName" , m_params.KmlMapPath);
  nh.getParam("/op_global_planner/ReedsShepp" , m_params.bEnableReedsShepp);
  nh.getParam("/op_global_planner/RoadEdge", m_params.bEnableRoadEdgePlanning);
  nh.getParam("/op_global_planner/DistanceToRoadEdge", distanceToRoadEdge);
  nh.getParam("/op_global_planner/goalDataPath",goalDataPath);
  std::cout<<goalDataPath<<std::endl;

  int iSource = 0;
  nh.getParam("/op_global_planner/mapSource", iSource);
  if(iSource == 0)
    m_params.mapSource = PlannerHNS::MAP_AUTOWARE;
  else if (iSource == 1)
    m_params.mapSource = PlannerHNS::MAP_FOLDER;
  else if(iSource == 2)
    m_params.mapSource = PlannerHNS::MAP_KML_FILE;

  tf::StampedTransform transform;
  PlannerHNS::ROSHelpers::GetTransformFromTF("map", "world", transform);
  m_OriginPos.position.x  = transform.getOrigin().x();
  m_OriginPos.position.y  = transform.getOrigin().y();
  m_OriginPos.position.z  = transform.getOrigin().z();

  pub_Paths = nh.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 1, true);
  pub_PathsRviz = nh.advertise<visualization_msgs::MarkerArray>("global_waypoints_rviz", 1, true);
  pub_MapRviz  = nh.advertise<visualization_msgs::MarkerArray>("vector_map_center_lines_rviz", 1, true);
  pub_GoalsListRviz = nh.advertise<visualization_msgs::MarkerArray>("op_destinations_rviz", 1, true);
  vehicle_status_pub_ = nh.advertise<autoware_msgs::DrivingStatus>("driving_status", 1);

  if(m_params.bEnableRvizInput)
  {
    sub_start_pose = nh.subscribe("/initialpose", 1, &GlobalPlanner::callbackGetStartPose, this);
    // sub_goal_pose = nh.subscribe("move_base_simple/goal", 1, &GlobalPlanner::callbackGetGoalPose, this);
    sub_scenario = nh.subscribe("scenario_manager/scenario_cmd",1,&GlobalPlanner::callbackGetScenario,this);
  }
  else
  {
    LoadSimulationData(goalDataPath);
  }

  sub_current_pose = nh.subscribe("/current_pose", 10, &GlobalPlanner::callbackGetCurrentPose, this);

  int bVelSource = 1;
  nh.getParam("/op_global_planner/velocitySource", bVelSource);
  if(bVelSource == 0)
    sub_robot_odom = nh.subscribe("/odom", 10, &GlobalPlanner::callbackGetRobotOdom, this);
  else if(bVelSource == 1)
    sub_current_velocity = nh.subscribe("/current_velocity", 10, &GlobalPlanner::callbackGetVehicleStatus, this);
  else if(bVelSource == 2)
    sub_can_info = nh.subscribe("/can_info", 10, &GlobalPlanner::callbackGetCANInfo, this);

  if(m_params.bEnableDynamicMapUpdate)
    sub_road_status_occupancy = nh.subscribe<>("/occupancy_road_status", 1, &GlobalPlanner::callbackGetRoadStatusOccupancyGrid, this);

  //Mapping Section
  sub_lanes = nh.subscribe("/vector_map_info/lane", 1, &GlobalPlanner::callbackGetVMLanes,  this);
  sub_points = nh.subscribe("/vector_map_info/point", 1, &GlobalPlanner::callbackGetVMPoints,  this);
  sub_dt_lanes = nh.subscribe("/vector_map_info/dtlane", 1, &GlobalPlanner::callbackGetVMdtLanes,  this);
  sub_intersect = nh.subscribe("/vector_map_info/cross_road", 1, &GlobalPlanner::callbackGetVMIntersections,  this);
  sup_area = nh.subscribe("/vector_map_info/area", 1, &GlobalPlanner::callbackGetVMAreas,  this);
  sub_lines = nh.subscribe("/vector_map_info/line", 1, &GlobalPlanner::callbackGetVMLines,  this);
  sub_stop_line = nh.subscribe("/vector_map_info/stop_line", 1, &GlobalPlanner::callbackGetVMStopLines,  this);
  sub_signals = nh.subscribe("/vector_map_info/signal", 1, &GlobalPlanner::callbackGetVMSignal,  this);
  sub_vectors = nh.subscribe("/vector_map_info/vector", 1, &GlobalPlanner::callbackGetVMVectors,  this);
  sub_curbs = nh.subscribe("/vector_map_info/curb", 1, &GlobalPlanner::callbackGetVMCurbs,  this);
  sub_edges = nh.subscribe("/vector_map_info/road_edge", 1, &GlobalPlanner::callbackGetVMRoadEdges,  this);
  sub_way_areas = nh.subscribe("/vector_map_info/way_area", 1, &GlobalPlanner::callbackGetVMWayAreas,  this);
  sub_cross_walk = nh.subscribe("/vector_map_info/cross_walk", 1, &GlobalPlanner::callbackGetVMCrossWalks,  this);
  sub_nodes = nh.subscribe("/vector_map_info/node", 1, &GlobalPlanner::callbackGetVMNodes,  this);

}

GlobalPlanner::~GlobalPlanner()
{
  if(m_params.bEnableRvizInput)
    SaveSimulationData();
}

void GlobalPlanner::callbackGetRoadStatusOccupancyGrid(const nav_msgs::OccupancyGridConstPtr& msg)
{
//  std::cout << "Occupancy Grid Origin (" << msg->info.origin.position.x << ", " << msg->info.origin.position.x << ") , " << msg->header.frame_id << ", Res: " << msg->info.resolution <<  std::endl;

  m_GridMapIntType.clear();

  //std::cout << "Found Map Data: Zero " <<  std::endl;
  for(unsigned int i=0; i < msg->data.size(); i++)
  {
    if((int8_t)msg->data.at(i) == 0)
      m_GridMapIntType.push_back(0);
    else if((int8_t)msg->data.at(i) == 50)
      m_GridMapIntType.push_back(75);
    else if((int8_t)msg->data.at(i) == 100)
      m_GridMapIntType.push_back(255);
    else
      m_GridMapIntType.push_back(128);

      //std::cout << msg->data.at(i) << ",";
  }
  //std::cout << std::endl << "--------------------------------------------------------" << std::endl;

  //std::cout << "Found Map Data: Zero : " << m_GridMapIntType.size() <<  std::endl;
  PlannerHNS::WayPoint center(msg->info.origin.position.x, msg->info.origin.position.y, msg->info.origin.position.z, tf::getYaw(msg->info.origin.orientation));
  PlannerHNS::OccupancyToGridMap grid(msg->info.width,msg->info.height, msg->info.resolution, center);
  std::vector<PlannerHNS::WayPoint*> modified_nodes;
  timespec t;
  UtilityHNS::UtilityH::GetTickCount(t);
  PlannerHNS::MappingHelpers::UpdateMapWithOccupancyGrid(grid, m_GridMapIntType, m_Map, modified_nodes);
  m_ModifiedMapItemsTimes.push_back(std::make_pair(modified_nodes, t));

  visualization_msgs::MarkerArray map_marker_array;
  PlannerHNS::ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);

//  visualization_msgs::Marker mkr = PlannerHNS::ROSHelpers::CreateGenMarker(center.pos.x, center.pos.y, center.pos.z, 0, 0,0,1,0.5, 1000, "TestCenter", visualization_msgs::Marker::SPHERE);
//
//  map_marker_array.markers.push_back(mkr);

  pub_MapRviz.publish(map_marker_array);
}

void GlobalPlanner::ClearOldCostFromMap()
{
  for(int i=0; i < (int)m_ModifiedMapItemsTimes.size(); i++)
  {
    if(UtilityHNS::UtilityH::GetTimeDiffNow(m_ModifiedMapItemsTimes.at(i).second) > CLEAR_COSTS_TIME)
    {
      for(unsigned int j= 0 ; j < m_ModifiedMapItemsTimes.at(i).first.size(); j++)
      {
        for(unsigned int i_action=0; i_action < m_ModifiedMapItemsTimes.at(i).first.at(j)->actionCost.size(); i_action++)
        {
          if(m_ModifiedMapItemsTimes.at(i).first.at(j)->actionCost.at(i_action).first == PlannerHNS::FORWARD_ACTION)
          {
            m_ModifiedMapItemsTimes.at(i).first.at(j)->actionCost.at(i_action).second = 0;
          }
        }
      }

      m_ModifiedMapItemsTimes.erase(m_ModifiedMapItemsTimes.begin()+i);
      i--;
    }
  }
}

void GlobalPlanner::callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  PlannerHNS::WayPoint wp = PlannerHNS::WayPoint(msg->pose.position.x+m_OriginPos.position.x, msg->pose.position.y+m_OriginPos.position.y, msg->pose.position.z+m_OriginPos.position.z, tf::getYaw(msg->pose.orientation));
  m_GoalsPos.push_back(wp);
  ROS_INFO("Received Goal Pose");
}

void GlobalPlanner::callbackGetStartPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  m_CurrentPose = PlannerHNS::WayPoint(msg->pose.pose.position.x+m_OriginPos.position.x, msg->pose.pose.position.y+m_OriginPos.position.y, msg->pose.pose.position.z+m_OriginPos.position.z, tf::getYaw(msg->pose.pose.orientation));
  ROS_INFO("Received Start pose");
}

void GlobalPlanner::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  m_CurrentPose = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
}

void GlobalPlanner::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
  m_VehicleState.speed = msg->twist.twist.linear.x;
  m_CurrentPose.v = m_VehicleState.speed;
  if(fabs(msg->twist.twist.linear.x) > 0.25)
    m_VehicleState.steer += atan(2.7 * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
}

void GlobalPlanner::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
  m_VehicleState.speed = msg->twist.linear.x;
  m_CurrentPose.v = m_VehicleState.speed;
  if(fabs(msg->twist.linear.x) > 0.25)
    m_VehicleState.steer = atan(2.7 * msg->twist.angular.z/msg->twist.linear.x);
  UtilityHNS::UtilityH::GetTickCount(m_VehicleState.tStamp);
}

void GlobalPlanner::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
  m_VehicleState.speed = msg->speed/3.6;
  m_CurrentPose.v = m_VehicleState.speed;
  m_VehicleState.steer = msg->angle * 0.45 / 660;
  UtilityHNS::UtilityH::GetTickCount(m_VehicleState.tStamp);
}

bool GlobalPlanner::GenerateGlobalPlan(PlannerHNS::WayPoint& startPoint, PlannerHNS::WayPoint& goalPoint, std::vector<std::vector<PlannerHNS::WayPoint> >& generatedTotalPaths)
{
  std::vector<int> predefinedLanesIds;
  double ret = 0;

  ret = m_PlannerH.PlanUsingDP(startPoint, goalPoint, MAX_GLOBAL_PLAN_DISTANCE, m_params.bEnableLaneChange, predefinedLanesIds, m_Map, generatedTotalPaths);

  if(ret == 0)
  {
    std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString()
                        << ") and Goal (" << goalPoint.pos.ToString() << ")" << std::endl;
    return false;
  }


  if(generatedTotalPaths.size() > 0 && generatedTotalPaths.at(0).size()>0)
  {
    if(m_params.bEnableSmoothing)
    {
      for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
      {
        PlannerHNS::PlanningHelpers::FixPathDensity(generatedTotalPaths.at(i), m_params.pathDensity);
        PlannerHNS::PlanningHelpers::SmoothPath(generatedTotalPaths.at(i), 0.49, 0.35 , 0.01);
      }
    }

    for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
    {
      PlannerHNS::PlanningHelpers::CalcAngleAndCost(generatedTotalPaths.at(i));
      if(m_GlobalPathID > 10000)
        m_GlobalPathID = 1;

      for(unsigned int j=0; j < generatedTotalPaths.at(i).size(); j++)
        generatedTotalPaths.at(i).at(j).gid = m_GlobalPathID;

      m_GlobalPathID++;

      std::cout << "New DP Path -> " << generatedTotalPaths.at(i).size() << std::endl;

      // 还要额外发布一个end_point_status = 0，因为在a*场景的最后，这个标志位发布了2，在mpc_follower中对2进行了停车处理。所以在op planner开始的时候要把这个标志位重新置为0
      end_point_status_.vehicle_status.data = 0;
      vehicle_status_pub_.publish(end_point_status_);

    }
    return true;
  }
  else
  {
    std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString() << ") and Goal (" << goalPoint.pos.ToString() << ")" << std::endl;
  }
  return false;
}

double calPointDistance(const PointXYZ &a, const PointXYZ &b)
{
    return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

double calVecAngle(const Direction front, const Direction back)
{
    double v1=atan2(front.y, front.x);
    double v2=atan2(back.y, back.x);
    double cross_angle = v1-v2;
    // cout<<"before="<<cross_angle<<endl;
    if(cross_angle>M_PI)
    {
        cross_angle-=2.0*M_PI;
    }
    else if (cross_angle<-M_PI)
    {
        cross_angle+=2.0*M_PI;
    }
    // cout<<"after="<<cross_angle*180/M_PI<<endl;
    return cross_angle;
    
}

void removeUturn(vector<PointXYZ> &points)
{
    vector<Direction> GlobalPathDirection;
    Direction d;
    double kappa;
    for(unsigned int i=0; i<points.size()-1; i++)
    {
        d.x = points.at(i+1).x-points.at(i).x;
        d.y = points.at(i+1).y-points.at(i).y;
        GlobalPathDirection.push_back(d);
    }
    // vector<double> cross_angle_list;
    vector<int> mask;
    double cross_angle;
    for(unsigned int i=0; i<GlobalPathDirection.size()-1; i++)
    {
        cross_angle = calVecAngle(GlobalPathDirection.at(i+1), GlobalPathDirection.at(i));
        if(cross_angle>0.17453292519943295)
        {
            mask.push_back(i);
        }
    }

    cout<<"mask size()="<<mask.size()<<endl;
    int n=0;
    for(auto &ma:mask)
    {
        cout<<ma<<", ";
        n+=1;
        if(n%10==0)
        {
            cout<<endl;
        }
    }
    double start_idx;
    double end_idx;
    bool start_flag=false;
    bool end_flag=false;
    
    for(int i=mask.size()-1; i>0; --i)
    {
        if(i==mask.size()-1||!start_flag)
        {
            start_idx = mask.at(i);
            start_flag = true;
        }
        int interval = mask.at(i)-mask.at(i-1);
        // cout<<
        if(interval > 5)
        {
            end_idx = mask.at(i-1);
            end_flag = true;
        }
        if(start_flag&&end_flag)
        {
            if(abs(calVecAngle(GlobalPathDirection.at(start_idx), GlobalPathDirection.at(end_idx)))>2.356194490192345)
            {
                points.erase(points.begin()+end_idx, points.begin()+start_idx);
            }
            start_flag=false;
            end_flag=false;
            
        }
    }
    cout<<"after remove uturn points size"<<points.size()<<endl;
}



void reedsSheppPlan(const PointXYZ &start, const PointXYZ &end, vector<PointXYZ> &path)
{
    double q0[3]={start.x, start.y, tf::getYaw(start.a)};
    double q1[3]={end.x, end.y, tf::getYaw(end.a)};
    ReedsSheppStateSpace   *r=new ReedsSheppStateSpace;
    ReedsSheppStateSpace::ReedsSheppPath *rr=new ReedsSheppStateSpace::ReedsSheppPath;
    vector<vector<double > >finalpath;
    PointXYZ p;
    finalpath=r->xingshensample(q0,q1,1.5);
    for(int i=0;i<finalpath.size();i++)
    {
        // cout<<finalpath[i][0]<<" "<<finalpath[i][1]<<" "<<finalpath[i][2]<<endl;
        p.x = finalpath[i][0];
        p.y = finalpath[i][1];
        p.z = start.z;
        p.a = tf::createQuaternionMsgFromYaw(finalpath[i][2]);
        path.push_back(p);
    }
    cout<<"rs path generate successfully! with points is "<<path.size()<<endl;
}

void getWaypoints(vector<PointXYZ> &points, const autoware_msgs::Lane &lane)
{
  int num =lane.waypoints.size();
  for(int i=0; i<num; ++i)
  {
    PointXYZ p;
    p.x = lane.waypoints[i].pose.pose.position.x;
    p.y = lane.waypoints[i].pose.pose.position.y;
    p.z = lane.waypoints[i].pose.pose.position.z;
    p.a = lane.waypoints[i].pose.pose.orientation;
    points.push_back(p);
  }
}

void genReedsSheppPath(vector<PointXYZ> &points)
{
  vector<vector<PointXYZ>> RS_points;
  vector<PointXYZ> RS_pair;
  vector<int> RS_index;
  for(int i=0; i<points.size()-1; i++)
  {
    double temp = calPointDistance(points.at(i), points.at(i+1));
    if(temp>=8.0)
    {
      RS_pair.push_back(points.at(i));
      RS_pair.push_back(points.at(i+1));
      RS_points.push_back(RS_pair);
      RS_index.push_back(i);
      RS_pair.clear();
    }   
  }

  cout<<"rs pair size="<<RS_points.size()<<endl;
  vector<PointXYZ> reedsSheppPath;
  for(int i=RS_points.size()-1; i>=0; i--)
  {
    reedsSheppPlan(RS_points.at(i).at(0), RS_points.at(i).at(1), reedsSheppPath);
    points.insert(points.begin()+RS_index.at(i)+1, reedsSheppPath.begin(), reedsSheppPath.end());
    reedsSheppPath.clear();
  }
}

void exchangeXY(std::vector<vector_map::Point> &pt_vector)
{
    double temp = pt_vector.at(0).bx;
    pt_vector.at(0).bx=pt_vector.at(0).ly;
    pt_vector.at(0).ly=temp;
    temp = pt_vector.at(1).bx;
    pt_vector.at(1).bx=pt_vector.at(1).ly;
    pt_vector.at(1).ly=temp;
}

bool GlobalPlanner::findPointInLine(const vector_map::Line &line,std::vector<vector_map::Point> &pt_vector)
{
    if(line.lid == 0)
       return false;
    pt_vector.clear();
    for(auto &pt:roadEdge_vmap.points)
    {
        if(line.bpid != pt.pid && line.fpid != pt.pid)
           continue;
        pt_vector.push_back(pt);
    }

    if(pt_vector.size()!=2)
    {
        std::cout<<"Can not find Points in Line!"<<std::endl;
        return false;
    }
    return true;
}

void GlobalPlanner::getRoadEdgeLines(vector<vector<PointXYZ>> &road_edge_lines)
{
     for(auto &line:roadEdge_vmap.lines)
    {
        if(line.lid == 0)
           continue;
        std::vector<vector_map::Point> pt_vector;
        vector<PointXYZ> lines;
        PointXYZ start;
        PointXYZ end;
 
        if(findPointInLine(line, pt_vector))
        {
            exchangeXY(pt_vector);
            start.x=pt_vector.at(0).bx;
            start.y=pt_vector.at(0).ly;
            start.z=0;
            end.x=pt_vector.at(1).bx;
            end.y=pt_vector.at(1).ly;
            end.z=0;
            lines.push_back(start);
            lines.push_back(end);
            road_edge_lines.push_back(lines);
            lines.clear();
        }
    }
    cout<<"=================================="<<endl;
    cout<<" the num of road_edge_lines is "<<road_edge_lines.size()<<endl;
}


void GlobalPlanner::create_roadEdge_vmap()
{
    if(all_vmap.points.empty() || all_vmap.lines.empty()||all_vmap.road_edges.empty())
       return;
    for(auto &road_edge:all_vmap.road_edges)
    {
        roadEdge_vmap.road_edges.push_back(road_edge);
        for(auto &line:all_vmap.lines)
        {
            if(line.lid != road_edge.lid)
               continue;
            roadEdge_vmap.lines.push_back(line);
            for(auto &point:all_vmap.points)
            {
                if(line.bpid == point.pid)
                   roadEdge_vmap.points.push_back(point);
                if(line.flid == 0 && line.fpid == point.pid)
                   roadEdge_vmap.points.push_back(point);
            }
        }
    }
    std::cout<<"after filter, vmap points: "<<roadEdge_vmap.points.size()<<std::endl;
    std::cout<<"after filter, vmap lines: "<<roadEdge_vmap.lines.size()<<std::endl;
    std::cout<<"after filter, vmap edges: "<<roadEdge_vmap.road_edges.size()<<std::endl;
}

double sgn(double x)
{
    if(x==0)
    {
        return 0.0;
    }
    else if(x>0)
    {
        return 1.0;
    }
    else{
        return -1.0;
    }
}

double calcu_norm(vector<double> vec)
{
    return sqrt(vec[0]*vec[0]+vec[1]*vec[1]);
}

double InnerProduct(vector<double> m, vector<double> n)
{
    return m[0]*n[0]+m[1]*n[1];
}

vector<double> uintilize(vector<double> vec)
{
    vector<double> p;
    double norm=calcu_norm(vec);
    p.push_back(vec[0]/norm);
    p.push_back(vec[1]/norm);
    return p;
}

double vecCross(vector<double> m, vector<double> n)
{
    return m[0]*n[1]-m[1]*n[0];
}

void GlobalPlanner::calc_GlobalNearPath(vector<vector<PointXYZ>> road_edge_lines, 
                                        vector<PointXYZ> points,
                                        double pathToEdge, 
                                        vector<PointXYZ> &near_path)
{
    cout<<"distance to edge ="<<pathToEdge<<endl;
    
    for(auto &p:points)
    {
        vector<double> h_set;
        vector<vector<PointXYZ>> h_set_lines;
        vector<double> cross_set;
        double px;
        double py;
        double h;
        double cross;
        double step;
        double yaw = tf::getYaw(p.a);
        vector<double> p_dir{cos(yaw), sin(yaw)};
        vector<double> line_dir;
        vector<double> line_angle_list;
        for(auto &line:road_edge_lines)
        {
            vector<double> FL{line[1].x-line[0].x, line[1].y-line[0].y};
            vector<double> PF{line[0].x-p.x, line[0].y-p.y};
            vector<double> PL{line[1].x-p.x, line[1].y-p.y};
            double FL_norm = calcu_norm(FL);
            if(FL_norm < 1e-2)
                continue;
            if(InnerProduct(p_dir, FL)>0)
            {
                if(vecCross(PF, PL)<0){continue;}
                line_dir.clear();
                line_dir.push_back(FL[0]);
                line_dir.push_back(FL[1]);
            }
            else{
                if(vecCross(PL, PF)<0){continue;}
                line_dir.clear();
                line_dir.push_back(-1.0*FL[0]);
                line_dir.push_back(-1.0*FL[1]);
            }

            if(InnerProduct(FL, PF)*-1.0<0)
            {
                continue;
            }
            else if(InnerProduct(FL, PL)<0)
            {
                continue;
            }
            else
            {
                double line_angle = atan2(line_dir[1], line_dir[0]);
                cross=vecCross(PF, PL);
                h = abs(cross)/FL_norm;
                if(h>8.0){continue;}
                line_angle_list.push_back(line_angle);
                cross_set.push_back(cross);
                h_set.push_back(h); 
                h_set_lines.push_back(line);  
            }
        }

        if(h_set.size()==0)
        {
            continue;
        }
        int minPosition = min_element(h_set.begin(),h_set.end()) - h_set.begin();
        double h_min = h_set[minPosition];
        step = h_min-pathToEdge;
        double dline_x = h_set_lines[minPosition].at(1).x-h_set_lines[minPosition].at(0).x;
        double dline_y = h_set_lines[minPosition].at(1).y-h_set_lines[minPosition].at(0).y;
        vector<double> min_line{dline_x, dline_y};
        vector<double> unit(uintilize(min_line));
        double x_step = sgn(cross_set[minPosition])*unit[1]*step;
        double y_step = -1.0*sgn(cross_set[minPosition])*unit[0]*step;
        px = p.x + x_step;
        py = p.y + y_step;

        PointXYZ final_p;
        final_p.x=px;
        final_p.y=py;
        final_p.z=p.z;
        geometry_msgs::Quaternion a = tf::createQuaternionMsgFromYaw(line_angle_list[minPosition]);
        final_p.a=a;
        near_path.push_back(final_p);
    }

    vector<vector<PointXYZ>> RS_points;
    vector<PointXYZ> RS_pair;
    vector<int> RS_index;
    
    for(int i=0; i<near_path.size()-1; i++)
    {
        
        double temp = calPointDistance(near_path.at(i), near_path.at(i+1));
        if(temp>=8.0)
        {
            RS_pair.push_back(near_path.at(i));
            RS_pair.push_back(near_path.at(i+1));
            RS_points.push_back(RS_pair);
            RS_index.push_back(i);
            RS_pair.clear();
        }   
    }
    cout<<"rs pair size="<<RS_points.size()<<endl;
    vector<PointXYZ> reedsSheppPath;
    for(int i=RS_points.size()-1; i>=0; i--)
    {
        reedsSheppPlan(RS_points.at(i).at(0), RS_points.at(i).at(1), reedsSheppPath);
        near_path.insert(near_path.begin()+RS_index.at(i) + 1, reedsSheppPath.begin(), reedsSheppPath.end());
        reedsSheppPath.clear();
    }
    // pubNeighborLane(near_path);
    cout<<"near path size ="<<near_path.size()<<endl;
    cout<<"Global Path of near edge with neighbour road edge points have been generated!"<<endl;    
}


void GlobalPlanner::VisualizeAndSend(const std::vector<std::vector<PlannerHNS::WayPoint> > generatedTotalPaths)
{
  autoware_msgs::LaneArray lane_array;
  visualization_msgs::MarkerArray pathsToVisualize;

  for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
  {
    autoware_msgs::Lane lane;
    PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(generatedTotalPaths.at(i), lane);
    if(m_params.bEnableReedsShepp)
    {
      cout<<"starting rs planning!!!!"<<endl;
      vector<PointXYZ> points;
      getWaypoints(points, lane);
      removeUturn(points);
      genReedsSheppPath(points);
      autoware_msgs::Lane result_lane;
      for(unsigned int i=0; i < points.size(); i++)
      {
          autoware_msgs::Waypoint wp = lane.waypoints.at(0);
          wp.pose.pose.position.x = points.at(i).x;
          wp.pose.pose.position.y = points.at(i).y;
          wp.pose.pose.position.z = points.at(i).z;
          wp.pose.pose.orientation = points.at(i).a;
          result_lane.waypoints.push_back(wp);
      }
      lane_array.lanes.push_back(result_lane);  
    }
    else
    {
      lane_array.lanes.push_back(lane);
    }
  }
  // road edge path --note by yuxiang
  // if lane size >= 2 cout error, but to be continue.
  if(lane_array.lanes.size()>=2)
  {
    cout<<"Error:  the size of lane_array is larger than 2"<<endl;
    return;
  }

  if(m_params.bEnableRoadEdgePlanning)
  {
    vector<vector<PointXYZ>> road_edge_lines;
    vector<PointXYZ> global_path;
    getWaypoints(global_path,lane_array.lanes[0]);
    getRoadEdgeLines(road_edge_lines);
    vector<PointXYZ> near_path;
    calc_GlobalNearPath(road_edge_lines, global_path, distanceToRoadEdge, near_path);
    autoware_msgs::Lane near_lane;
    for(unsigned int i=0; i < near_path.size(); i++)
    {
        autoware_msgs::Waypoint wp = lane_array.lanes.at(0).waypoints.at(0);
        wp.pose.pose.position.x = near_path.at(i).x;
        wp.pose.pose.position.y = near_path.at(i).y;
        wp.pose.pose.position.z = near_path.at(i).z;
        wp.pose.pose.orientation = near_path.at(i).a;
        near_lane.waypoints.push_back(wp);
    }
    lane_array.lanes.push_back(near_lane);
  }

  std_msgs::ColorRGBA total_color;
  total_color.r = 0;
  total_color.g = 0.7;
  total_color.b = 1.0;
  total_color.a = 0.9;
  PlannerHNS::ROSHelpers::createGlobalLaneArrayMarker(total_color, lane_array, pathsToVisualize);
  PlannerHNS::ROSHelpers::createGlobalLaneArrayOrientationMarker(lane_array, pathsToVisualize);
  PlannerHNS::ROSHelpers::createGlobalLaneArrayVelocityMarker(lane_array, pathsToVisualize);
  pub_PathsRviz.publish(pathsToVisualize);
  if((m_bFirstStart && m_params.bEnableHMI) || !m_params.bEnableHMI)
    pub_Paths.publish(lane_array);

  for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
  {
    std::ostringstream str_out;
    str_out << UtilityHNS::UtilityH::GetHomeDirectory();
    str_out << UtilityHNS::DataRW::LoggingMainfolderName;
    str_out << UtilityHNS::DataRW::GlobalPathLogFolderName;
    str_out << "GlobalPath_";
    str_out << i;
    str_out << "_";
    PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(), generatedTotalPaths.at(i));
  } 
}

void GlobalPlanner::VisualizeDestinations(std::vector<PlannerHNS::WayPoint>& destinations, const int& iSelected)
{
  visualization_msgs::MarkerArray goals_array;

  for(unsigned int i=0; i< destinations.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "HMI_Destinations";
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 3.25;
    marker.scale.y = 3.25;
    marker.scale.z = 3.25;
    marker.color.a = 0.9;
    marker.id = i;
    if(i == iSelected)
    {
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;
    }
    else
    {
      marker.color.r = 0.2;
      marker.color.g = 0.8;
      marker.color.b = 0.2;
    }
    marker.pose.position.x = destinations.at(i).pos.x;
    marker.pose.position.y = destinations.at(i).pos.y;
    marker.pose.position.z = destinations.at(i).pos.z;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(destinations.at(i).pos.a);

    std::ostringstream str_out;
    str_out << "G";
    marker.text = str_out.str();

    goals_array.markers.push_back(marker);
  }
  pub_GoalsListRviz.publish(goals_array);
}

void GlobalPlanner::SaveSimulationData()
{
  std::vector<std::string> simulationDataPoints;
  std::ostringstream startStr;
  startStr << m_CurrentPose.pos.x << "," << m_CurrentPose.pos.y << "," << m_CurrentPose.pos.z << "," << m_CurrentPose.pos.a << ","<< m_CurrentPose.cost << "," << 0 << ",";
  simulationDataPoints.push_back(startStr.str());

  for(unsigned int i=0; i < m_GoalsPos.size(); i++)
  {
    std::ostringstream goalStr;
    goalStr << m_GoalsPos.at(i).pos.x << "," << m_GoalsPos.at(i).pos.y << "," << m_GoalsPos.at(i).pos.z << "," << m_GoalsPos.at(i).pos.a << "," << 0 << "," << 0 << ",destination_" << i+1 << ",";
    simulationDataPoints.push_back(goalStr.str());
  }

  std::string header = "X,Y,Z,A,C,V,name,";

  std::ostringstream fileName;
  fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::SimulationFolderName;
  fileName << "EgoCar.csv";
  std::ofstream f(fileName.str().c_str());

  if(f.is_open())
  {
    if(header.size() > 0)
      f << header << "\r\n";
    for(unsigned int i = 0 ; i < simulationDataPoints.size(); i++)
      f << simulationDataPoints.at(i) << "\r\n";
  }

  f.close();
}

int GlobalPlanner::LoadSimulationData(const std::string &goalDataFilePath)
{
  // std::ostringstream fileName;
  // fileName << "EgoCar.csv";

  // std::string simuDataFileName = UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::SimulationFolderName + fileName.str();
  
  UtilityHNS::SimulationFileReader sfr(goalDataFilePath);
  
  UtilityHNS::SimulationFileReader::SimulationData data;

  int nData = sfr.ReadAllData(data);
  if(nData == 0)
    return 0;

  m_CurrentPose = PlannerHNS::WayPoint(data.startPoint.x, data.startPoint.y, data.startPoint.z, data.startPoint.a);
  m_GoalsPos.push_back(PlannerHNS::WayPoint(data.goalPoint.x, data.goalPoint.y, data.goalPoint.z, data.goalPoint.a));

  for(unsigned int i=0; i < data.simuCars.size(); i++)
  {
    m_GoalsPos.push_back(PlannerHNS::WayPoint(data.simuCars.at(i).x, data.simuCars.at(i).y, data.simuCars.at(i).z, data.simuCars.at(i).a));
  }

  return nData;
}

void GlobalPlanner::MainLoop()
{
  ros::Rate loop_rate(25);
  timespec animation_timer;
  UtilityHNS::UtilityH::GetTickCount(animation_timer);

  while (ros::ok())
  {
    ros::spinOnce();
    bool bMakeNewPlan = false;

    if(m_params.mapSource == PlannerHNS::MAP_KML_FILE && !m_bKmlMap)
    {
      m_bKmlMap = true;
      PlannerHNS::MappingHelpers::LoadKML(m_params.KmlMapPath, m_Map);
      visualization_msgs::MarkerArray map_marker_array;
      PlannerHNS::ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);
      pub_MapRviz.publish(map_marker_array);
    }
    else if (m_params.mapSource == PlannerHNS::MAP_FOLDER && !m_bKmlMap)
    {
      m_bKmlMap = true;
      PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(m_params.KmlMapPath, m_Map, true);
      visualization_msgs::MarkerArray map_marker_array;
      PlannerHNS::ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);

      pub_MapRviz.publish(map_marker_array);
    }
    else if (m_params.mapSource == PlannerHNS::MAP_AUTOWARE && !m_bKmlMap)
    {
      std::vector<UtilityHNS::AisanDataConnFileReader::DataConn> conn_data;;

      if(m_MapRaw.GetVersion()==2)
      {
        std::cout << "Map Version 2" << endl;
        m_bKmlMap = true;
        PlannerHNS::MappingHelpers::ConstructRoadNetworkFromROSMessageV2(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
            m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
            m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,  m_MapRaw.pSignals->m_data_list,
            m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
            m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,
            m_MapRaw.pLanes, m_MapRaw.pPoints, m_MapRaw.pNodes, m_MapRaw.pLines, PlannerHNS::GPSPoint(), m_Map, true, m_params.bEnableLaneChange, false);
      }
      else if(m_MapRaw.GetVersion()==1)
      {
        std::cout << "Map Version 1" << endl;
        m_bKmlMap = true;
        PlannerHNS::MappingHelpers::ConstructRoadNetworkFromROSMessage(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
            m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
            m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,  m_MapRaw.pSignals->m_data_list,
            m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
            m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,  PlannerHNS::GPSPoint(), m_Map, true, m_params.bEnableLaneChange, false);
      }

      if(m_bKmlMap)
      {
        visualization_msgs::MarkerArray map_marker_array;
        PlannerHNS::ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);
        pub_MapRviz.publish(map_marker_array);
      }
    }

    ClearOldCostFromMap();

    if(m_GoalsPos.size() > 0 && activate_scenario)
    {
      if(m_GeneratedTotalPaths.size() > 0 && m_GeneratedTotalPaths.at(0).size() > 3)
      {
        if(m_params.bEnableReplanning)
        {
          PlannerHNS::RelativeInfo info;
          bool ret = PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_GeneratedTotalPaths, m_CurrentPose, 0.75, info);
          if(ret == true && info.iGlobalPath >= 0 &&  info.iGlobalPath < m_GeneratedTotalPaths.size() && info.iFront > 0 && info.iFront < m_GeneratedTotalPaths.at(info.iGlobalPath).size())
          {
            double remaining_distance =    m_GeneratedTotalPaths.at(info.iGlobalPath).at(m_GeneratedTotalPaths.at(info.iGlobalPath).size()-1).cost - (m_GeneratedTotalPaths.at(info.iGlobalPath).at(info.iFront).cost + info.to_front_distance);
            if(remaining_distance <= REPLANNING_DISTANCE)
            {
              bMakeNewPlan = true;
              if(m_GoalsPos.size() > 0)
                m_iCurrentGoalIndex = (m_iCurrentGoalIndex + 1) % m_GoalsPos.size();
              std::cout << "Current Goal Index = " << m_iCurrentGoalIndex << std::endl << std::endl;
            }
          }
        }
      }
      else
        bMakeNewPlan = true;

      if(bMakeNewPlan || (m_params.bEnableDynamicMapUpdate && UtilityHNS::UtilityH::GetTimeDiffNow(m_ReplnningTimer) > REPLANNING_TIME))
      {
        UtilityHNS::UtilityH::GetTickCount(m_ReplnningTimer);
        PlannerHNS::WayPoint goalPoint = m_GoalsPos.at(m_iCurrentGoalIndex);
        bool bNewPlan = GenerateGlobalPlan(m_CurrentPose, goalPoint, m_GeneratedTotalPaths);

        if(bNewPlan)
        {
          bMakeNewPlan = false;
          VisualizeAndSend(m_GeneratedTotalPaths);
        }
      }
      VisualizeDestinations(m_GoalsPos, m_iCurrentGoalIndex);
    }

    loop_rate.sleep();
  }
}


//Mapping Section

void GlobalPlanner::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
  std::cout << "Received Lanes" << msg.data.size() << endl;
  if(m_MapRaw.pLanes == nullptr)
    m_MapRaw.pLanes = new UtilityHNS::AisanLanesFileReader(msg);
}

void GlobalPlanner::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
  std::cout << "Received Points" << msg.data.size() << endl;
  if(m_MapRaw.pPoints  == nullptr)
    m_MapRaw.pPoints = new UtilityHNS::AisanPointsFileReader(msg);
  // std::cout << "Received Points" << msg.data.size() << std::endl;
  if(!msg.data.empty())
  {
      all_vmap.points = msg.data;
      create_roadEdge_vmap();
  }
}

void GlobalPlanner::callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg)
{
  std::cout << "Received dtLanes" << msg.data.size() << endl;
  if(m_MapRaw.pCenterLines == nullptr)
    m_MapRaw.pCenterLines = new UtilityHNS::AisanCenterLinesFileReader(msg);
}

void GlobalPlanner::callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg)
{
  std::cout << "Received CrossRoads" << msg.data.size() << endl;
  if(m_MapRaw.pIntersections == nullptr)
    m_MapRaw.pIntersections = new UtilityHNS::AisanIntersectionFileReader(msg);
}

void GlobalPlanner::callbackGetVMAreas(const vector_map_msgs::AreaArray& msg)
{
  std::cout << "Received Areas" << msg.data.size() << endl;
  if(m_MapRaw.pAreas == nullptr)
    m_MapRaw.pAreas = new UtilityHNS::AisanAreasFileReader(msg);
}

void GlobalPlanner::callbackGetVMLines(const vector_map_msgs::LineArray& msg)
{
  std::cout << "Received Lines" << msg.data.size() << endl;
  if(m_MapRaw.pLines == nullptr)
    m_MapRaw.pLines = new UtilityHNS::AisanLinesFileReader(msg);

  if(!msg.data.empty())
  {
      all_vmap.lines = msg.data;
      create_roadEdge_vmap();
  }
}

void GlobalPlanner::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
  std::cout << "Received StopLines" << msg.data.size() << endl;
  if(m_MapRaw.pStopLines == nullptr)
    m_MapRaw.pStopLines = new UtilityHNS::AisanStopLineFileReader(msg);
}

void GlobalPlanner::callbackGetVMSignal(const vector_map_msgs::SignalArray& msg)
{
  std::cout << "Received Signals" << msg.data.size() << endl;
  if(m_MapRaw.pSignals  == nullptr)
    m_MapRaw.pSignals = new UtilityHNS::AisanSignalFileReader(msg);
}

void GlobalPlanner::callbackGetVMVectors(const vector_map_msgs::VectorArray& msg)
{
  std::cout << "Received Vectors" << msg.data.size() << endl;
  if(m_MapRaw.pVectors  == nullptr)
    m_MapRaw.pVectors = new UtilityHNS::AisanVectorFileReader(msg);
}

void GlobalPlanner::callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg)
{
  std::cout << "Received Curbs" << msg.data.size() << endl;
  if(m_MapRaw.pCurbs == nullptr)
    m_MapRaw.pCurbs = new UtilityHNS::AisanCurbFileReader(msg);
}

void GlobalPlanner::callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg)
{
  std::cout << "Received Edges" << msg.data.size() << endl;
  if(m_MapRaw.pRoadedges  == nullptr)
    m_MapRaw.pRoadedges = new UtilityHNS::AisanRoadEdgeFileReader(msg);
  if(!msg.data.empty())
  {
      all_vmap.road_edges = msg.data;
      create_roadEdge_vmap();
  }
}

void GlobalPlanner::callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg)
{
  std::cout << "Received Wayareas" << msg.data.size() << endl;
  if(m_MapRaw.pWayAreas  == nullptr)
    m_MapRaw.pWayAreas = new UtilityHNS::AisanWayareaFileReader(msg);
}

void GlobalPlanner::callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg)
{
  std::cout << "Received CrossWalks" << msg.data.size() << endl;
  if(m_MapRaw.pCrossWalks == nullptr)
    m_MapRaw.pCrossWalks = new UtilityHNS::AisanCrossWalkFileReader(msg);
}

void GlobalPlanner::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
  std::cout << "Received Nodes" << msg.data.size() << endl;
  if(m_MapRaw.pNodes == nullptr)
    m_MapRaw.pNodes = new UtilityHNS::AisanNodesFileReader(msg);
}

// 场景管理节点发布出来的场景信息 在 op global planner中订阅，判断是否要开始触发op规划
void GlobalPlanner::callbackGetScenario(const autoware_msgs::ScenarioCmd &msg){
  m_GeneratedTotalPaths.clear(); // for multi test;
  m_GoalsPos.clear();
  
  if(msg.scenario == 0){
    activate_scenario = true;
    PlannerHNS::WayPoint wp = PlannerHNS::WayPoint(msg.goal.pose.position.x+m_OriginPos.position.x, msg.goal.pose.position.y+m_OriginPos.position.y, msg.goal.pose.position.z+m_OriginPos.position.z, tf::getYaw(msg.goal.pose.orientation));
    m_GoalsPos.push_back(wp);
    ROS_INFO("receieve lane driving scenario in op_global_planner!");
    // // 还要额外发布一个end_point_status = 0，因为在a*场景的最后，这个标志位发布了2，在mpc_follower中对2进行了停车处理。所以在op planner开始的时候要把这个标志位重新置为0
    // end_point_status_.vehicle_status.data = 0;
    // vehicle_status_pub_.publish(end_point_status_);
  }
  else if(msg.mode == 1){
    activate_scenario = false;
    ROS_INFO("receieve parking scenario command, shutdown lane driving in op_global_planner!");
  }

  else if(msg.mode == 0)
    activate_scenario = true;
}

}
