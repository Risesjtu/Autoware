#include <ros/ros.h>
#include <vector>
// #include 
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <rs.h>

// #include "op_planner/PlannerH.h"
// #include "op_planner/PlannerCommonDef.h"
// #include "op_planner/MappingHelpers.h"

#include <vector_map/vector_map.h>
#include <autoware_msgs/LaneArray.h>
#include "op_ros_helpers/op_ROSHelpers.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

// #include <roadedge_analyse/Lane.h>
// #include <roadedge_analyse/LaneArray.h>
// #include <roadedge_analyse/Waypoint.h>
// #include <roadedge_analyse/LaneType.h>
// #include <roadedge_analyse/DTLane.h>


using namespace std;

struct PointXYZ{
    double x;
    double y;
    double z;
    
    // double ox;
    // double oy;
    // double oz;
    // double ow;
    geometry_msgs::Quaternion a;
    // Point(double x=0, double y=0, double z=0)
    // {
    //     this->x=x;
    //     this->y=y;
    //     this->z=z;
    // }
};

struct Direction
{
    double x;
    double y;
};


struct VectorRoadEdge {
	std::vector<vector_map::Point> points;
    std::vector<vector_map::Line> lines;
    std::vector<vector_map::RoadEdge> road_edges;
};


typedef std::map<double,vector_map::Point> RoadEdgeCenter;

class roadEdgeAnaly
{
private:
    //ROS messages (topics)
	ros::NodeHandle nh;

	//define publishers
	ros::Publisher pub_CurrentPoseRviz;
    ros::Publisher pub_RoadEdgeRviz;
    ros::Publisher pub_CenterPoint;
    ros::Publisher pub_neighborLaneRviz;

	// define subscribers.
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_current_velocity;
    ros::Subscriber sub_points;
    ros::Subscriber sub_lines;
    ros::Subscriber sub_edges;
    ros::Subscriber sub_lane_points_array;

    PlannerHNS::WayPoint m_CurrentPos;
    PlannerHNS::VehicleState m_VehicleStatus;

    void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
    void callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg);

    //map_info
    VectorRoadEdge all_vmap;
    VectorRoadEdge roadEdge_vmap;
    autoware_msgs::LaneArray lane_;
    // vector<LaneType> lane_types;
    // LaneType lane_type;
    autoware_msgs::LaneArray edge_path;
    // vector<double> kappa;
    vector<PointXYZ> points;
    vector<PointXYZ> points_copy;
    vector<int> start_end_idx_list;
    double distance_to_road_edge;
    double rsStartToEndDistance;


    void callbackGetVMPoints(const vector_map::PointArray& msg);
	void callbackGetVMLines(const vector_map::LineArray& msg);
	void callbackGetVMRoadEdges(const vector_map::RoadEdgeArray& msg);
    void callbackGetLanePointsArray(const autoware_msgs::LaneArray::ConstPtr& larray);

    void pubPose(const PlannerHNS::WayPoint &pose);
    void pubRodeEdge(const std::vector<vector_map::Point> &neighbor_pts);
    void pubCenterPoint(const RoadEdgeCenter &map);
    // void pubNeighborLane(const vector<vector<double>> &near_path,const vector<double> &vec_z);
    void pubNeighborLane(const vector<PointXYZ> &near_path);

    double computeDist(const PlannerHNS::WayPoint &vehicle, const vector_map::Point &mid_pt);
    bool findPointInLine(const vector_map::Line &line,std::vector<vector_map::Point> &pt_vector);
    void create_roadEdge_vmap();
    void parseRoadEdge();
    void getWayPoints();
    // void calKappa(double kappa);
    void removeUturn();
    void removeClosePoint();
    void generateGlobalNearEdgePath();
    void getRoadEdgeLines(vector<vector<PointXYZ>> &road_edge_lines);
    // void getNeighbourPointIdx(vector<vector<vector<double>>>& road_edge_lines,
    //                         vector<int>& have_neighbour_edge_points_idx,
    //                         vector<int>& global_path_points_to_road_edge_line_idx,
    //                         vector<int>& global_path_points_to_road_edge_line_point_idx,
    //                         vector<int>& no_neighbour_edge_points_idx);
    // void calc_GlobalNearPath(vector<vector<double>> &near_path, 
    //                         vector<vector<vector<double>>> &road_edge_lines,
    //                         vector<int> &have_neighbour_edge_points_idx,
    //                         vector<int> &global_path_points_to_road_edge_line_idx,
    //                         vector<int> &global_path_points_to_road_edge_line_point_idx);
    void calc_GlobalNearPath(vector<vector<PointXYZ>> road_edge_lines, 
                                        vector<PointXYZ> points,
                                        double pathToEdge, 
                                        vector<PointXYZ> &near_path);
    

    // bool findNearRoadEdge();
    // void createEdgePath();
    // void adjoinPath();
    vector_map::Point midPoint(const vector_map::Point &pt1, const vector_map::Point &pt2);
    


public:
    roadEdgeAnaly(/* args */);
    ~roadEdgeAnaly();
    void MainLoop();
};

// double calPointDistance(const PointXYZ &a, const PointXYZ &b);

