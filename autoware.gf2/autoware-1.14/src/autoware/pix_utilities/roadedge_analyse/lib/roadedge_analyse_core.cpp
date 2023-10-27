#include "roadedge_analyse/roadedge_analyse_core.h"
#include <vector>
using namespace std;

roadEdgeAnaly::roadEdgeAnaly()
{
    //订阅当前车辆位置和速度
    sub_current_pose = nh.subscribe("/current_pose", 10, &roadEdgeAnaly::callbackGetCurrentPose, this);
    sub_current_velocity = nh.subscribe("/current_velocity", 10, &roadEdgeAnaly::callbackGetVehicleStatus, this);
    
    //订阅VectorMap信息
    sub_points = nh.subscribe("/vector_map_info/point", 1, &roadEdgeAnaly::callbackGetVMPoints,  this);
    sub_lines = nh.subscribe("/vector_map_info/line", 1, &roadEdgeAnaly::callbackGetVMLines,  this);
    sub_edges = nh.subscribe("/vector_map_info/road_edge", 1, &roadEdgeAnaly::callbackGetVMRoadEdges,  this);
    sub_lane_points_array = nh.subscribe("/lane_waypoints_array", 1, &roadEdgeAnaly::callbackGetLanePointsArray, this);

    //发布车辆位置
    pub_CurrentPoseRviz = nh.advertise<visualization_msgs::Marker>("pix_roadedge_test", 1);
    pub_RoadEdgeRviz = nh.advertise<visualization_msgs::Marker>("neighbor_roadedge",1);
    pub_CenterPoint = nh.advertise<visualization_msgs::MarkerArray>("center_points",1);
    pub_neighborLaneRviz = nh.advertise<visualization_msgs::MarkerArray>("neighbor_Lane",1);

    nh.param("distance_to_road_edge", distance_to_road_edge, 1.5);
    nh.param("rsStartToEndDistance", rsStartToEndDistance, 1.5);
    nh.param("roadEdgeGenDistanceThreshold", roadEdgeGenDistanceThreshold, 8.0);

}

roadEdgeAnaly::~roadEdgeAnaly()
{

}

bool roadEdgeAnaly::findPointInLine(const vector_map::Line &line,std::vector<vector_map::Point> &pt_vector)
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


void roadEdgeAnaly::parseRoadEdge()
{
    double min_dist = 1e8;
    vector_map::Line neighbor_line;
    std::vector<vector_map::Point> neighbor_pts;
    RoadEdgeCenter dis_pt_map;
    if(roadEdge_vmap.lines.empty())
    {
        std::cout<<"roadEdge_vmap do not contain lines!"<<std::endl;
        return;
    }

    for(auto &line:roadEdge_vmap.lines)
    {
        if(line.lid == 0)
           continue;
        std::vector<vector_map::Point> pt_vector;
        if(findPointInLine(line,pt_vector))
        {
            vector_map::Point mid_pt= midPoint(pt_vector.at(0),pt_vector.at(1));
            double temp_dist = computeDist(m_CurrentPos,mid_pt);
            dis_pt_map.insert(std::make_pair(temp_dist,mid_pt));
            if(temp_dist<min_dist)
            {
                min_dist = temp_dist;
                neighbor_line = line;
                neighbor_pts = pt_vector;
            }
        }
    }
    pubRodeEdge(neighbor_pts);
    pubCenterPoint(dis_pt_map);
}


double roadEdgeAnaly::computeDist(const PlannerHNS::WayPoint &vehicle, const vector_map::Point &mid_pt)
{

    double dx = vehicle.pos.x - mid_pt.ly;
    double dy = vehicle.pos.y - mid_pt.bx;

    return sqrt(dx*dx+dy*dy);

}

void roadEdgeAnaly::create_roadEdge_vmap()
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



void roadEdgeAnaly::pubPose(const PlannerHNS::WayPoint &pose)
{
    visualization_msgs::Marker pose_marker;

    pose_marker.header.frame_id = "map";
    pose_marker.header.stamp = ros::Time::now();
    pose_marker.id = 0;
    pose_marker.type = visualization_msgs::Marker::ARROW;
    pose_marker.action = visualization_msgs::Marker::ADD;
    pose_marker.lifetime = ros::Duration();

    pose_marker.color.r = 0.0f;
    pose_marker.color.b = 0.0f;
    pose_marker.color.g = 1.0f;
    pose_marker.color.a = 1.0;

    pose_marker.scale.x = 2;
    pose_marker.scale.y = 0.2;
    pose_marker.scale.z = 0.01;

    pose_marker.pose.position.x = pose.pos.x;
    pose_marker.pose.position.y = pose.pos.y;
    pose_marker.pose.position.z = pose.pos.z;
    pose_marker.pose.orientation = tf::createQuaternionMsgFromYaw(pose.pos.a);

    pub_CurrentPoseRviz.publish(pose_marker);

}

void roadEdgeAnaly::pubRodeEdge(const std::vector<vector_map::Point> &neighbor_pts)
{
    visualization_msgs::Marker line_marker;
    if(neighbor_pts.size()!=2)
    {
        std::cout<<"No neighbor points!"<<std::endl;
        return;
    }

    vector_map::Point bp = neighbor_pts.at(0);
    if (bp.pid == 0)
      return ;

    vector_map::Point fp = neighbor_pts.at(1);
    if (fp.pid == 0)
      return ;

    line_marker.points.push_back(vector_map::convertPointToGeomPoint(bp));
    line_marker.points.push_back(vector_map::convertPointToGeomPoint(fp));

    line_marker.header.frame_id = "map";
    line_marker.header.stamp = ros::Time::now();
    line_marker.id = 0;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.lifetime = ros::Duration();

    line_marker.scale.x = 1;
    line_marker.color = createColorRGBA(vector_map::Color::RED);

    pub_RoadEdgeRviz.publish(line_marker);
}

void roadEdgeAnaly::pubNeighborLane(const vector<PointXYZ> &near_path)
{
    visualization_msgs::MarkerArray pointsArray;
    autoware_msgs::LaneArray lane_array;
    visualization_msgs::MarkerArray pathsToVisualize;
    autoware_msgs::Lane lane;
    std::vector<PlannerHNS::WayPoint> path;
    // trajectory.waypoints.clear();

    for(unsigned int i=0; i < near_path.size(); i++)
    {
        autoware_msgs::Waypoint wp;
        wp.pose.pose.position.x = near_path.at(i).x;
        wp.pose.pose.position.y = near_path.at(i).y;
        wp.pose.pose.position.z = near_path.at(i).z;
        wp.pose.pose.orientation = near_path.at(i).a;
        lane.waypoints.push_back(wp);
    }
    PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(lane, path);
    PlannerHNS::PlanningHelpers::SmoothPath(path, 0.49, 0.35 , 0.01);
    autoware_msgs::Lane trajectory;
    PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(path, trajectory, 0);


    lane_array.lanes.push_back(trajectory);
    std_msgs::ColorRGBA total_color;
    total_color.r = 1.0;
    total_color.g = 0.0;
    total_color.b = 0.0;
    total_color.a = 0.9;
    PlannerHNS::ROSHelpers::createGlobalLaneArrayMarker(total_color, lane_array, pathsToVisualize);
    PlannerHNS::ROSHelpers::createGlobalLaneArrayOrientationMarker(lane_array, pathsToVisualize);
    PlannerHNS::ROSHelpers::createGlobalLaneArrayVelocityMarker(lane_array, pathsToVisualize);
    pub_neighborLaneRviz.publish(pathsToVisualize);
}

void roadEdgeAnaly::pubCenterPoint(const RoadEdgeCenter &map)
{
    int count = 0;
    visualization_msgs::MarkerArray pt_array;
    for(auto & element:map)
    {
        visualization_msgs::Marker text;
        text.header.frame_id = "map";
        text.header.stamp = ros::Time::now();
        text.id = count++;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.color = createColorRGBA(vector_map::Color::GREEN);
        text.pose.orientation.w = 1.0;
        text.text=std::to_string(element.first);
        text.scale.z = 1;

        geometry_msgs::Pose pose;
        pose.position.x = element.second.ly;
        pose.position.y = element.second.bx;
        pose.position.z = element.second.h;
        text.pose=pose;

        pt_array.markers.emplace_back(text);
    }
    pub_CenterPoint.publish(pt_array);
}

void roadEdgeAnaly::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
    pubPose(m_CurrentPos);
    parseRoadEdge();
}

void roadEdgeAnaly::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed;
}


// for vector_map info
void roadEdgeAnaly::callbackGetVMPoints(const vector_map::PointArray& msg)
{
	std::cout << "Received Points" << msg.data.size() << std::endl;
    if(!msg.data.empty())
    {
        all_vmap.points = msg.data;
        create_roadEdge_vmap();
    }
}

void roadEdgeAnaly::callbackGetVMLines(const vector_map::LineArray& msg)
{
	std::cout << "Received Lines" << msg.data.size() << std::endl;
    if(!msg.data.empty())
    {
       all_vmap.lines = msg.data;
       create_roadEdge_vmap();
    }
}

void roadEdgeAnaly::callbackGetVMRoadEdges(const vector_map::RoadEdgeArray& msg)
{
	std::cout << "Received Edges" << msg.data.size() << std::endl;
    if(!msg.data.empty())
    {
        all_vmap.road_edges = msg.data;
        create_roadEdge_vmap();
    }
}

vector_map::Point roadEdgeAnaly::midPoint(const vector_map::Point &pt1, const vector_map::Point &pt2)
{
    vector_map::Point mid_point;
    mid_point.bx = (pt1.bx + pt2.bx)/2;
    mid_point.ly = (pt1.ly + pt2.ly)/2;
    mid_point.h = (pt1.h + pt2.h)/2;
    return mid_point;
}


double cal_yaw_pointxyz(const PointXYZ &point)
{
    // float x=point.ox;
    // float y=point.oy;
    // float z=point.oz;
    // float w=point.ow;
    // geometry_msgs::Quaternion a;
    // a.x = x;
    // a.y = y;
    // a.z = z;
    // a.w = w;
    double yaw=tf::getYaw(point.a);
    if(yaw>M_PI)
    {
        yaw -= 2.0*M_PI;
    }
    else if(yaw<-1.0*M_PI)
    {
        yaw += 2.0*M_PI;
    }
    return yaw;
}

void deleteUnnormalPoints(vector<PointXYZ> &points)
{
    double mindist=0.1;
    cout<<"before delete unnormal points size="<<points.size()<<endl;
    for(int i=points.size()-2; i>0; --i)
    {
        double yaw1 = cal_yaw_pointxyz(points.at(i));
        double yaw2 = cal_yaw_pointxyz(points.at(i+1));
        double yaw3 = cal_yaw_pointxyz(points.at(i+1));
        if((fabs(yaw1-yaw2)>60.0/180.0*M_PI)||(fabs(yaw2-yaw3)>60.0/180.0*M_PI))
        {
            points.erase(points.begin()+i+1);
        }
    }
    cout<<"after delete unnormal points size="<<points.size()<<endl;
}


void roadEdgeAnaly::getWayPoints()
{
    autoware_msgs::LaneArray lane_array_copy(lane_);
    int num = lane_array_copy.lanes[0].waypoints.size();
    vector<double> pointx;
    vector<double> pointy;
    vector<double> pointz;
    cout<<"laney point array has "<<num<<" points"<<endl;
    // vector<Point> global_points;
    for(int i=0; i<num; ++i)
    {
      PointXYZ p;
      p.x=lane_array_copy.lanes[0].waypoints[i].pose.pose.position.x;
      p.y=lane_array_copy.lanes[0].waypoints[i].pose.pose.position.y;
      p.z=lane_array_copy.lanes[0].waypoints[i].pose.pose.position.z;
      p.a=lane_array_copy.lanes[0].waypoints[i].pose.pose.orientation;
      points.push_back(p);
    }
    points_copy = points;
    cout<<"points size is: "<<points.size()<<endl;
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

void exchangeXY(std::vector<vector_map::Point> &pt_vector)
{
    double temp = pt_vector.at(0).bx;
    pt_vector.at(0).bx=pt_vector.at(0).ly;
    pt_vector.at(0).ly=temp;
    temp = pt_vector.at(1).bx;
    pt_vector.at(1).bx=pt_vector.at(1).ly;
    pt_vector.at(1).ly=temp;
    
}

void roadEdgeAnaly::getRoadEdgeLines(vector<vector<PointXYZ>> &road_edge_lines)
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

double calPointDistance(const PointXYZ &a, const PointXYZ &b)
{
    return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
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
        p.x = finalpath[i][0];
        p.y = finalpath[i][1];
        p.z = start.z;
        p.a = tf::createQuaternionMsgFromYaw(finalpath[i][2]);
        path.push_back(p);
    }
    cout<<"rs path generate successfully! with points is "<<path.size()<<endl;
}

void roadEdgeAnaly::calc_GlobalNearPath(vector<vector<PointXYZ>> road_edge_lines, 
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
    pubNeighborLane(near_path);
    cout<<"near path size ="<<near_path.size()<<endl;
    cout<<"Global Path of near edge with neighbour road edge points have been generated!"<<endl;    
}



void roadEdgeAnaly::generateGlobalNearEdgePath()
{
    if(roadEdge_vmap.lines.empty())
    {
        std::cout<<"ERROR:roadEdge_vmap do not contain lines!"<<std::endl;
        return;
    }

    vector<vector<PointXYZ>> road_edge_lines;
    getRoadEdgeLines(road_edge_lines);
    vector<PointXYZ> near_path;
    calc_GlobalNearPath(road_edge_lines, points, distance_to_road_edge, near_path);
}

double calVecAngle(const Direction front, const Direction back)
{
    double v1=atan2(front.y, front.x);
    double v2=atan2(back.y, back.x);
    double cross_angle = v1-v2;
    if(cross_angle>M_PI)
    {
        cross_angle-=2.0*M_PI;
    }
    else if (cross_angle<-M_PI)
    {
        cross_angle+=2.0*M_PI;
    }
    return cross_angle;
    
}

void roadEdgeAnaly::removeUturn()
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
                start_end_idx_list.push_back(end_idx);
                start_end_idx_list.push_back(start_idx);
            }
            start_flag=false;
            end_flag=false;
            
        }
    }
    cout<<"after remove uturn points size"<<points.size()<<endl;
}

void roadEdgeAnaly::removeClosePoint()
{
    vector<vector<int>> closePointPair;
    vector<int> closePoint;
    vector<PointXYZ> points_copy(points);
    double distance;
    for(int i=0; i<points_copy.size(); ++i)
    {
        for(int j=i+1; j<points_copy.size(); ++j)
        {
            distance = calPointDistance(points_copy.at(i), points_copy.at(j));
            
            if(distance<0.1)
            {
                cout<<"point distance="<<distance<<endl;
                closePoint.push_back(i);
                closePoint.push_back(j);
                cout<<"("<<i<<", "<<j<<")"<<endl;
                closePointPair.push_back(closePoint);
                closePoint.clear();
            }
        }
    }
    cout<<"points pair size"<<closePointPair.size()<<endl;
}

void roadEdgeAnaly::callbackGetLanePointsArray(const autoware_msgs::LaneArray::ConstPtr& larray)
{
if (larray->lanes.empty())
{
    return;
}
lane_ = *larray;
roadEdgeAnaly::getWayPoints();
removeUturn();
roadEdgeAnaly::generateGlobalNearEdgePath();
}

void roadEdgeAnaly::MainLoop()
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

