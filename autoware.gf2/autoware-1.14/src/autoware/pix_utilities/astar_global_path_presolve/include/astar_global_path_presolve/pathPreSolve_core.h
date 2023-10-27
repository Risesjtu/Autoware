#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/LaneArray.h>
#include "libwaypoint_follower/libwaypoint_follower.h"
#include <unordered_map>
#include <ros/console.h>
#include <autoware_msgs/ScenarioCmd.h>
#include <autoware_msgs/DrivingStatus.h>



typedef std::unordered_map<int, std::pair<int, bool>> segRecord;
class pathPreSolve
{
private:
   ros::NodeHandle nh_;

   ros::Subscriber base_waypoints_sub_;
   ros::Subscriber current_pose_sub_;
   ros::Subscriber current_velocity_sub_;
   ros::Subscriber closest_waypoint_sub_;
   ros::Subscriber sub_scenario_;

   ros::Publisher base_waypoints_cut_pub_;
   ros::Publisher vehicle_status_pub_;
   ros::Publisher pub_ScenarioEnded_;

   segRecord pathSeg;
   int offset ;
   int closest_local_index_;
   int closest_waypoint_index_;
   autoware_msgs::DrivingStatus end_point_status;
   autoware_msgs::DrivingStatus prev_end_point_status;

   autoware_msgs::Lane trajectory_;
   autoware_msgs::Lane partial_trajectory_;
   std::vector<size_t> reversing_indices_;
   size_t target_index_ = 0;  // 如果target_index_不在一开始给一个值，会在第一次执行is_near_target = getPlaneDistance(trajectory_.waypoints.at(target_index_)时，报错数组越界
   size_t prev_target_index_ = 0;
   bool is_completed_;
   bool is_near_target;
   bool is_first_update_target = true;
   double arrived_distance_m;
   double stopped_velocity_mps;
   bool is_first_update_gear = true;
   bool is_ended_published;
   bool activate_scenario_;

   int is_switched = 0;  // debug

   void currentPoseCallback(const geometry_msgs::PoseStamped &msg);
   void currentVelocityCallback(const geometry_msgs::TwistStamped &msg);
   void baseWaypointsCallback(const autoware_msgs::Lane &msg);
   void closestWaypointCallback(const std_msgs::Int32& msg);
   void resetVelocity(autoware_msgs::Lane &original_waypoints);
   void callbackGetScenario(const autoware_msgs::ScenarioCmd &msg);

  
public:
  autoware_msgs::Lane base_waypoints_cut_;
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::TwistStamped current_velocity_;

  bool current_pose_initialized_;
  bool current_velocity_initialized_;
  bool closest_waypoint_initialized_;

  pathPreSolve();
  ~pathPreSolve();

  // int getLocalClosestWaypoint(const autoware_msgs::Lane& waypoints, const geometry_msgs::Pose& pose, int search_size = 30);
  void MainLoop();

  autoware_msgs::Lane getPartialTrajectory(const autoware_msgs::Lane & trajectory, const size_t start_index, const size_t end_index);
  std::vector<size_t> getReversingIndices(const autoware_msgs::Lane & trajectory);
  size_t getNextTargetIndex(const size_t trajectory_size, const std::vector<size_t> & reversing_indices, const size_t current_target_index);
  void updateTargetIndex();

};
