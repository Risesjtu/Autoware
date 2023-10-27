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
#include <visualization_msgs/Marker.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "op_ros_helpers/op_ROSHelpers.h"
#include <queue>
#include <std_msgs/Bool.h>
#include <unordered_map>

using namespace std;
using autoware_msgs::ScenarioCmd;

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

// 场景类，用于将某个场景结束后要执行的控制命令 和 场景绑定起来，利于管理和后续拓展
class Scenario{
  public:
    autoware_msgs::ScenarioCmd planning_cmd_;  // Scenario的属性之一：场景命令，用于将控制队列和对应的场景绑定
    // 控制命令是pair类型，first是string类型，标识这个控制的名字；second是控制命令的具体内容
    queue<std::pair<string,autoware_msgs::CleaningStatus>> control_cmd_que_;  // 控制队列

    Scenario() = default;
    ~Scenario() = default;
    
    Scenario(autoware_msgs::ScenarioCmd planning_cmd);  // 构造函数
    void set_planning_cmd(autoware_msgs::ScenarioCmd planning_cmd);  // 设置场景命令，表示控制队列所属的场景
    void push_control_cmd(std::pair<string,autoware_msgs::CleaningStatus> control_cmd);
    void pop_control_cmd();
};


class ScenarioManager{
private:
  //ROS messages (topics)
	ros::NodeHandle nh_;

  //define publishers
  ros::Publisher pub_scenario_;
  ros::Publisher pub_cleaning_cmd_;
  ros::Publisher pub_IsGoalEmpty;
  ros::Publisher rvizMarkerPub_;

  //define subscribers
  ros::Subscriber sub_rviz_goal_;
  ros::Subscriber sub_points_;
  ros::Subscriber sub_lines_;
  ros::Subscriber sub_area_;
  ros::Subscriber sub_park_area_;
  ros::Subscriber sub_lanes_;
  ros::Subscriber sub_nodes_;
  ros::Subscriber sub_goal_pose_;
  ros::Subscriber sub_current_pose_;
  ros::Subscriber sub_scenario_end_status;
  ros::Subscriber sub_cmd_feedback;

  ros::Subscriber sub_robot_odom_;
  ros::Subscriber sub_current_velocity_;
  ros::Subscriber sub_can_info_;
  ros::Subscriber sub_goal_from_loading_;

  ros::Timer timer;

  queue<Scenario> scenario_que_;
  queue<autoware_msgs::DockInfo> goal_que_; 
  PlannerHNS::WayPoint current_pose_;
  VectorCustomArea all_vmap_;

  // park area data
  vector<ParkArea> park_areas_;
  bool scenario_ended;

  bool start_pose_initialized_;
  double max_dist_to_lane_driving_goal;
  double min_dist_to_lane_driving_goal;
  bool enableCover;
  bool pre_scenario_published = false;
  bool pre_control_published = false;
  unordered_map<string,bool> control_cmd_feedback_;
  geometry_msgs::PoseStamped goal_from_loading_;
  std_msgs::Int32 is_goal_empty_;

  tf::TransformBroadcaster *tf_broadcaster;
  visualization_msgs::Marker rvizMarker_;

  void callbackGetRvizGoal(const autoware_msgs::DockInfoConstPtr &msg);
  void callbackGetVMPoints(const vector_map::PointArray& msg);
	void callbackGetVMLines(const vector_map::LineArray& msg);
	void callbackGetVMAreas(const vector_map::AreaArray& msg);
	void callbackGetVMCustomAreas(const vector_map::CustomAreaArray& msg);
  void callbackGetVMLanes(const vector_map::LaneArray &msg);
  void callbackGetVMNodes(const vector_map::NodeArray &msg);

  void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
  void callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg);
  void callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg);
  void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
  void callbackGetScenarioEnded(const std_msgs::Bool &msg);
  void callbackGetControlFeedback(const autoware_msgs::VehicleCleaningFeedbackConstPtr &msg);
  void callbackScenarioPub(const ros::TimerEvent& e);
  void callbackGetGoalFromLoading(const geometry_msgs::PoseStamped& msg);


  // create park area
  void create_park_area_vmap();

  // flag: 0 to find stop goal, 1 to find start goal
  bool find_nearest_goal_in_lane(const PlannerHNS::GPSPoint &rough_goal, geometry_msgs::PoseStamped &modified_goal,int flag);
  
  
  inline double calculate_distance(const PlannerHNS::GPSPoint &p1, const PlannerHNS::GPSPoint &p2){
    return std::sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
  };
  inline double calculate_distance(const PlannerHNS::GPSPoint &p1, const geometry_msgs::PoseStamped &p2){
    return std::sqrt((p1.x - p2.pose.position.x)*(p1.x - p2.pose.position.x) + (p1.y - p2.pose.position.y)*(p1.y - p2.pose.position.y));
  };
  autoware_msgs::ScenarioCmd  convert_to_world_pose(const autoware_msgs::ScenarioCmd &scenario);
  void deal_control_cmd(Scenario &current_scenario);


  void generate_scenario(autoware_msgs::DockInfo &current_goal, bool cover_flag);

  void VisualizeTF(geometry_msgs::PoseStamped pose, std::string task);

  void visualizeMarker(geometry_msgs::PoseStamped pose, std::string task);

public:
  geometry_msgs::Pose origin_pose_;
  const std::vector<std::vector<double> > cover_pose_{
    {-14.8, 39.2, 0, 0, 0, 1, 0},
    {-17.7, 29.3, 0, 0, 0, 0, 1}
  };
  ScenarioManager();
  ~ScenarioManager();
  void MainLoop();
};