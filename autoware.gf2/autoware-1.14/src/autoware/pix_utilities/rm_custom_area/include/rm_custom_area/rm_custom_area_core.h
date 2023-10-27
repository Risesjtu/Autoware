#include <ros/ros.h>
#include <vector>
#include <vector_map/vector_map.h>
#include "op_planner/PlannerH.h"
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/DockInfo.h>
#include <autoware_msgs/ScenarioCmd.h>
#include <autoware_msgs/CleaningStatus.h>
#include <autoware_msgs/DrivingStatus.h>
#include <autoware_msgs/VehicleCleaningFeedback.h>
#include <autoware_can_msgs/CANInfo.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "op_ros_helpers/op_ROSHelpers.h"
#include <queue>
#include <std_msgs/Bool.h>
#include <unordered_map>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>

using namespace std;

class ParkArea{
public:
    vector<PlannerHNS::GPSPoint> vertexs_;

    ParkArea(){};
    ~ParkArea(){};
    bool isInside(const PlannerHNS::GPSPoint &rough_goal);
    void push_back(const PlannerHNS::GPSPoint &point);
    
};

struct VectorCustomArea{
  std::vector<vector_map::Point> points;
  std::vector<vector_map::Line> lines;
  std::vector<vector_map::Area> areas;
  std::vector<vector_map::CustomArea> custom_areas;
  std::vector<vector_map::Lane> lanes;
  std::vector<vector_map::Node> nodes;
};



class KillStopLine{
private:
  //ROS messages (topics)
	ros::NodeHandle nh_;

  //define publishers
  ros::Publisher pub_pointcloud_;

  //define subscribers
  ros::Subscriber sub_custom_area_;
  ros::Subscriber sub_origin_pc_;
  ros::Subscriber sub_current_pose_;
  ros::Subscriber sub_scenario_cmd_;
  ros::Subscriber sub_points_;
  ros::Subscriber sub_lines_;
  ros::Subscriber sub_area_;
  ros::Subscriber sub_lanes_;
  ros::Subscriber sub_nodes_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_;
  VectorCustomArea all_vmap_;
  PlannerHNS::WayPoint current_pose_;
  bool scenario_stop_line_;
  vector<ParkArea> park_areas_;
  tf::StampedTransform transform_;

  void callbackGetVMCustomAreas(const vector_map::CustomAreaArray& msg);
  void callbackOriginPC(const sensor_msgs::PointCloud2::ConstPtr msg);
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackGetScenario(const autoware_msgs::ScenarioCmd &msg);

  void callbackGetVMPoints(const vector_map::PointArray& msg);
	void callbackGetVMLines(const vector_map::LineArray& msg);
	void callbackGetVMAreas(const vector_map::AreaArray& msg);
  void callbackGetVMLanes(const vector_map::LaneArray &msg);
  void callbackGetVMNodes(const vector_map::NodeArray &msg);

  void create_park_area_vmap();

  void RemoveCustomArea(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, double max_x, double max_y, double min_x, double min_y,
                                       pcl::PointCloud<pcl::PointXYZI>::Ptr out_filtered_cloud_ptr, bool remove_indices);

public:

  KillStopLine();
  ~KillStopLine();
  void MainLoop();
};