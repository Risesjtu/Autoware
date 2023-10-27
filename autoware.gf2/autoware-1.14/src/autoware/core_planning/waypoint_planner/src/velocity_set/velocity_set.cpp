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

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <iostream>

#include <waypoint_planner/velocity_set/libvelocity_set.h>
#include <waypoint_planner/velocity_set/velocity_set_info.h>
#include <waypoint_planner/velocity_set/velocity_set_path.h>

namespace
{
constexpr int LOOP_RATE = 10;
constexpr double DECELERATION_SEARCH_DISTANCE = 30;
constexpr double STOP_SEARCH_DISTANCE = 60;

void obstacleColorByKind(const EControl kind, std_msgs::ColorRGBA &color, const double alpha=0.5)
{
  if (kind == EControl::STOP)
  {
    color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = alpha;  // red
  }
  else if (kind == EControl::STOPLINE)
  {
    color.r = 0.0; color.g = 0.0; color.b = 1.0; color.a = alpha;  // blue
  }
  else if (kind == EControl::DECELERATE)
  {
    color.r = 1.0; color.g = 1.0; color.b = 0.0; color.a = alpha;  // yellow
  }
  else
  {
    color.r = 1.0; color.g = 1.0; color.b = 1.0; color.a = alpha;  // white
  }
}

// Display a detected obstacle
void displayObstacle(const EControl& kind, const ObstaclePoints& obstacle_points, const ros::Publisher& obstacle_pub)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  static geometry_msgs::Point prev_obstacle_point;
  if (kind == EControl::STOP || kind == EControl::STOPLINE || kind == EControl::DECELERATE)
  {
    marker.pose.position = obstacle_points.getObstaclePoint(kind);
    prev_obstacle_point = marker.pose.position;
  }
  else  // kind == OTHERS
  {
    marker.pose.position = prev_obstacle_point;
  }
  geometry_msgs::Quaternion quat;
  marker.pose.orientation = quat;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 2.0;
  marker.lifetime = ros::Duration(0.1);
  marker.frame_locked = true;
  obstacleColorByKind(kind, marker.color, 0.7);

  obstacle_pub.publish(marker);
}

void displayDetectionRange(const autoware_msgs::Lane& lane, const CrossWalk& crosswalk, const int closest_waypoint,
                           const EControl& kind, const int obstacle_waypoint, const double stop_range,
                           const double deceleration_range, const ros::Publisher& detection_range_pub)
{
  // set up for marker array
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker crosswalk_marker;
  visualization_msgs::Marker waypoint_marker_stop;
  visualization_msgs::Marker waypoint_marker_decelerate;
  visualization_msgs::Marker stop_line;
  crosswalk_marker.header.frame_id = "/map";
  crosswalk_marker.header.stamp = ros::Time();
  crosswalk_marker.id = 0;
  crosswalk_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  crosswalk_marker.action = visualization_msgs::Marker::ADD;
  waypoint_marker_stop = crosswalk_marker;
  waypoint_marker_decelerate = crosswalk_marker;
  stop_line = crosswalk_marker;
  stop_line.type = visualization_msgs::Marker::CUBE;

  // set each namespace
  crosswalk_marker.ns = "Crosswalk Detection";
  waypoint_marker_stop.ns = "Stop Detection";
  waypoint_marker_decelerate.ns = "Decelerate Detection";
  stop_line.ns = "Stop Line";

  // set scale and color
  double scale = 2 * stop_range;
  waypoint_marker_stop.scale.x = scale;
  waypoint_marker_stop.scale.y = scale;
  waypoint_marker_stop.scale.z = scale;
  waypoint_marker_stop.color.a = 0.2;
  waypoint_marker_stop.color.r = 0.0;
  waypoint_marker_stop.color.g = 1.0;
  waypoint_marker_stop.color.b = 0.0;
  waypoint_marker_stop.frame_locked = true;

  scale = 2 * (stop_range + deceleration_range);
  waypoint_marker_decelerate.scale.x = scale;
  waypoint_marker_decelerate.scale.y = scale;
  waypoint_marker_decelerate.scale.z = scale;
  waypoint_marker_decelerate.color.a = 0.15;
  waypoint_marker_decelerate.color.r = 1.0;
  waypoint_marker_decelerate.color.g = 1.0;
  waypoint_marker_decelerate.color.b = 0.0;
  waypoint_marker_decelerate.frame_locked = true;

  if (obstacle_waypoint > -1)
  {
    stop_line.pose.position = lane.waypoints[obstacle_waypoint].pose.pose.position;
    stop_line.pose.orientation = lane.waypoints[obstacle_waypoint].pose.pose.orientation;
  }
  stop_line.pose.position.z += 1.0;
  stop_line.scale.x = 0.1;
  stop_line.scale.y = 15.0;
  stop_line.scale.z = 2.0;
  stop_line.lifetime = ros::Duration(0.1);
  stop_line.frame_locked = true;
  obstacleColorByKind(kind, stop_line.color, 0.3);

  int crosswalk_id = crosswalk.getDetectionCrossWalkID();
  if (crosswalk_id > 0)
    scale = crosswalk.getDetectionPoints(crosswalk_id).width;
  crosswalk_marker.scale.x = scale;
  crosswalk_marker.scale.y = scale;
  crosswalk_marker.scale.z = scale;
  crosswalk_marker.color.a = 0.5;
  crosswalk_marker.color.r = 0.0;
  crosswalk_marker.color.g = 1.0;
  crosswalk_marker.color.b = 0.0;
  crosswalk_marker.frame_locked = true;

  // set marker points coordinate
  for (int i = 0; i < STOP_SEARCH_DISTANCE; i++)
  {
    if (closest_waypoint < 0 || i + closest_waypoint > static_cast<int>(lane.waypoints.size()) - 1)
      break;

    geometry_msgs::Point point;
    point = lane.waypoints[closest_waypoint + i].pose.pose.position;

    waypoint_marker_stop.points.push_back(point);

    if (i > DECELERATION_SEARCH_DISTANCE)
      continue;
    waypoint_marker_decelerate.points.push_back(point);
  }

  if (crosswalk_id > 0)
  {
    if (!crosswalk.isMultipleDetection())
    {
      for (const auto& p : crosswalk.getDetectionPoints(crosswalk_id).points)
        crosswalk_marker.points.push_back(p);
    }
    else
    {
      for (const auto& c_id : crosswalk.getDetectionCrossWalkIDs())
      {
        for (const auto& p : crosswalk.getDetectionPoints(c_id).points)
        {
          scale = crosswalk.getDetectionPoints(c_id).width;
          crosswalk_marker.points.push_back(p);
        }
      }
    }
  }
  // publish marker
  marker_array.markers.push_back(crosswalk_marker);
  marker_array.markers.push_back(waypoint_marker_stop);
  marker_array.markers.push_back(waypoint_marker_decelerate);
  if (kind != EControl::KEEP)
    marker_array.markers.push_back(stop_line);
  detection_range_pub.publish(marker_array);
  marker_array.markers.clear();
}

// obstacle detection for crosswalk
EControl crossWalkDetection(const pcl::PointCloud<pcl::PointXYZ>& points, const CrossWalk& crosswalk,
                            const geometry_msgs::PoseStamped& localizer_pose, const int points_threshold,
                            ObstaclePoints* obstacle_points)
{
  int crosswalk_id = crosswalk.getDetectionCrossWalkID();
  double search_radius = crosswalk.getDetectionPoints(crosswalk_id).width / 2;
  // std::vector<int> crosswalk_ids crosswalk.getDetectionCrossWalkIDs();

  // Search each calculated points in the crosswalk
  for (const auto& c_id : crosswalk.getDetectionCrossWalkIDs())
  {
    for (const auto& p : crosswalk.getDetectionPoints(c_id).points)
    {
      geometry_msgs::Point detection_point = calcRelativeCoordinate(p, localizer_pose.pose);
      tf::Vector3 detection_vector = point2vector(detection_point);
      detection_vector.setZ(0.0);

      int stop_count = 0;  // the number of points in the detection area
      for (const auto& p : points)
      {
        tf::Vector3 point_vector(p.x, p.y, 0.0);
        double distance = tf::tfDistance(point_vector, detection_vector);
        if (distance < search_radius)
        {
          stop_count++;
          geometry_msgs::Point point_temp;
          point_temp.x = p.x;
          point_temp.y = p.y;
          point_temp.z = p.z;
          obstacle_points->setStopPoint(calcAbsoluteCoordinate(point_temp, localizer_pose.pose));
        }
        if (stop_count > points_threshold)
          return EControl::STOP;
      }
    }

    obstacle_points->clearStopPoints();
    if (!crosswalk.isMultipleDetection())
      break;
  }
  return EControl::KEEP;  // find no obstacles
}

// 判断是否有障碍物 在正常轨迹上（障碍物 直接用 “点云数量超过阈值就算作是障碍物”）
int detectStopObstacle(const pcl::PointCloud<pcl::PointXYZ>& points, const int closest_waypoint,
                       const autoware_msgs::Lane& lane, const CrossWalk& crosswalk, double stop_range,
                       double points_threshold, const geometry_msgs::PoseStamped& localizer_pose,
                       ObstaclePoints* obstacle_points, EObstacleType* obstacle_type,
                       const int wpidx_detection_result_by_other_nodes)
{
  int stop_obstacle_waypoint = -1;
  *obstacle_type = EObstacleType::NONE;
  // start search from the closest waypoint
  for (int i = closest_waypoint; i < closest_waypoint + STOP_SEARCH_DISTANCE; i++)
  {
    // reach the end of waypoints
    if (i >= static_cast<int>(lane.waypoints.size()))
      break;

    // detection another nodes
    if (wpidx_detection_result_by_other_nodes >= 0 &&
        lane.waypoints.at(i).gid == wpidx_detection_result_by_other_nodes)
    {
      stop_obstacle_waypoint = i;
      *obstacle_type = EObstacleType::STOPLINE;
      obstacle_points->setStopPoint(lane.waypoints.at(i).pose.pose.position); // for vizuialization
      break;
    }

    // Detection for cross walk
    if (i == crosswalk.getDetectionWaypoint())
    {
      // found an obstacle in the cross walk
      if (crossWalkDetection(points, crosswalk, localizer_pose, points_threshold, obstacle_points) == EControl::STOP)
      {
        stop_obstacle_waypoint = i;
        *obstacle_type = EObstacleType::ON_CROSSWALK;
        break;
      }
    }

    // waypoint seen by localizer
    geometry_msgs::Point waypoint = calcRelativeCoordinate(lane.waypoints[i].pose.pose.position, localizer_pose.pose);
    tf::Vector3 tf_waypoint = point2vector(waypoint);
    tf_waypoint.setZ(0);

    int stop_point_count = 0;
    for (const auto& p : points)
    {
      tf::Vector3 point_vector(p.x, p.y, 0);

      // 2D distance between waypoint and points (obstacle)
      double dt = tf::tfDistance(point_vector, tf_waypoint);
      if (dt < stop_range)
      {
        stop_point_count++;
        geometry_msgs::Point point_temp;
        point_temp.x = p.x;
        point_temp.y = p.y;
        point_temp.z = p.z;
        obstacle_points->setStopPoint(calcAbsoluteCoordinate(point_temp, localizer_pose.pose));
      }
    }

    // there is an obstacle if the number of points exceeded the threshold
    if (stop_point_count > points_threshold)
    {
      stop_obstacle_waypoint = i;
      *obstacle_type = EObstacleType::ON_WAYPOINTS;
      break;
    }

    obstacle_points->clearStopPoints();

    // check next waypoint...
  }

  return stop_obstacle_waypoint;
}

int detectDecelerateObstacle(const pcl::PointCloud<pcl::PointXYZ>& points, const int closest_waypoint,
                             const autoware_msgs::Lane& lane, const double stop_range, const double deceleration_range,
                             const double points_threshold, const geometry_msgs::PoseStamped& localizer_pose,
                             ObstaclePoints* obstacle_points)
{
  int decelerate_obstacle_waypoint = -1;
  // start search from the closest waypoint
  for (int i = closest_waypoint; i < closest_waypoint + DECELERATION_SEARCH_DISTANCE; i++)
  {
    // reach the end of waypoints
    if (i >= static_cast<int>(lane.waypoints.size()))
      break;

    // waypoint seen by localizer
    geometry_msgs::Point waypoint = calcRelativeCoordinate(lane.waypoints[i].pose.pose.position, localizer_pose.pose);
    tf::Vector3 tf_waypoint = point2vector(waypoint);
    tf_waypoint.setZ(0);

    int decelerate_point_count = 0;
    for (const auto& p : points)
    {
      tf::Vector3 point_vector(p.x, p.y, 0);

      // 2D distance between waypoint and points (obstacle)
      double dt = tf::tfDistance(point_vector, tf_waypoint);
      if (dt > stop_range && dt < stop_range + deceleration_range)
      {
        decelerate_point_count++;
        geometry_msgs::Point point_temp;
        point_temp.x = p.x;
        point_temp.y = p.y;
        point_temp.z = p.z;
        obstacle_points->setDeceleratePoint(calcAbsoluteCoordinate(point_temp, localizer_pose.pose));
      }
    }

    // there is an obstacle if the number of points exceeded the threshold
    if (decelerate_point_count > points_threshold)
    {
      decelerate_obstacle_waypoint = i;
      break;
    }

    obstacle_points->clearDeceleratePoints();

    // check next waypoint...
  }

  return decelerate_obstacle_waypoint;
}

// Detect an obstacle by using pointcloud
EControl pointsDetection(const pcl::PointCloud<pcl::PointXYZ>& points, const int closest_waypoint,
                         const autoware_msgs::Lane& lane, const CrossWalk& crosswalk, const VelocitySetInfo& vs_info,
                         int* obstacle_waypoint, ObstaclePoints* obstacle_points)
{
  // no input for detection || no closest waypoint
  if ((points.empty() == true && vs_info.getDetectionResultByOtherNodes() == -1) || closest_waypoint < 0)
    return EControl::KEEP;  // 如果 没有障碍物，就 返回 KEEP 状态

  // 障碍物的状态 EObstacleType，有 NONE, ON_WAYPOINTS, ON_CROSSWALK, STOPLINE
  EObstacleType obstacle_type = EObstacleType::NONE;
  // detect stop obstacle，返回obstacle的索引（如果有）
  int stop_obstacle_waypoint =
      detectStopObstacle(points, closest_waypoint, lane, crosswalk, vs_info.getStopRange(),
                         vs_info.getPointsThreshold(), vs_info.getLocalizerPose(),
                         obstacle_points, &obstacle_type, vs_info.getDetectionResultByOtherNodes());

  // skip searching deceleration range
  // 现阶段deceleration_range_是0，所以直接进下面的if
  if (vs_info.getDecelerationRange() < 0.01)  //  getDecelerationRange()返回deceleration_range_，表示“if obstacle is in this range, decelerate”
  {
    *obstacle_waypoint = stop_obstacle_waypoint;
    if (stop_obstacle_waypoint < 0)
      return EControl::KEEP;  // 如果 没有障碍物，就 返回 KEEP 状态
    else if (obstacle_type == EObstacleType::ON_WAYPOINTS || obstacle_type == EObstacleType::ON_CROSSWALK)
      return EControl::STOP;  // 如果 障碍物在正常轨迹waypoints或者crosswalk上，就 返回 STOP 状态
    else if (obstacle_type == EObstacleType::STOPLINE)
      return EControl::STOPLINE;  // 如果 有stopline，就 返回 STOPLINE 状态
    else
      return EControl::OTHERS;
  }

  // 下面的操作 返回的结果 和 上面的if分支 是一样的，就是下面的状态判断方法更复杂一些，会考虑到减速的情况。但现阶段没用下面的！
  int decelerate_obstacle_waypoint =
      detectDecelerateObstacle(points, closest_waypoint, lane, vs_info.getStopRange(), vs_info.getDecelerationRange(),
                               vs_info.getPointsThreshold(), vs_info.getLocalizerPose(), obstacle_points);

  // stop obstacle was not found
  if (stop_obstacle_waypoint < 0)
  {
    *obstacle_waypoint = decelerate_obstacle_waypoint;
    return decelerate_obstacle_waypoint < 0 ? EControl::KEEP : EControl::DECELERATE;
  }

  // stop obstacle was found but decelerate obstacle was not found
  if (decelerate_obstacle_waypoint < 0)
  {
    *obstacle_waypoint = stop_obstacle_waypoint;
    return EControl::STOP;
  }

  // about 5.0 meter
  double waypoint_interval =
      getPlaneDistance(lane.waypoints[0].pose.pose.position, lane.waypoints[1].pose.pose.position);
  int stop_decelerate_threshold = 5 / waypoint_interval;

  // both were found
  if (stop_obstacle_waypoint - decelerate_obstacle_waypoint > stop_decelerate_threshold)
  {
    *obstacle_waypoint = decelerate_obstacle_waypoint;
    return EControl::DECELERATE;
  }
  else
  {
    *obstacle_waypoint = stop_obstacle_waypoint;
    return EControl::STOP;
  }
}

// Detect obstacles by using pointcloud
EControl obstacleDetection(int closest_waypoint, const autoware_msgs::Lane& lane, const CrossWalk& crosswalk,
                           const VelocitySetInfo vs_info, const ros::Publisher& detection_range_pub,
                           const ros::Publisher& obstacle_pub, int* obstacle_waypoint)
{
  ObstaclePoints obstacle_points;
  // 根据多种检测（有障碍物检测、crosswalk检测等），更新 状态机 的状态，有三种状态：KEEP, STOP, DECELERATE
  EControl detection_result = pointsDetection(vs_info.getPoints(), closest_waypoint, lane, crosswalk, vs_info,
                                              obstacle_waypoint, &obstacle_points);
  displayDetectionRange(lane, crosswalk, closest_waypoint, detection_result, *obstacle_waypoint, vs_info.getStopRange(),
                        vs_info.getDecelerationRange(), detection_range_pub);

  static int false_count = 0;
  static EControl prev_detection = EControl::KEEP;
  static int prev_obstacle_waypoint = -1;

  // stop or decelerate because we found obstacles
  if (detection_result == EControl::STOP || detection_result == EControl::STOPLINE || detection_result == EControl::DECELERATE)
  {
    displayObstacle(detection_result, obstacle_points, obstacle_pub);
    prev_detection = detection_result;
    false_count = 0;
    prev_obstacle_waypoint = *obstacle_waypoint;
    return detection_result;
  }

  // there are no obstacles, but wait a little for safety
  if (prev_detection == EControl::STOP || prev_detection == EControl::STOPLINE || prev_detection == EControl::DECELERATE)
  {
    false_count++;

    if (false_count < LOOP_RATE / 2)
    {
      *obstacle_waypoint = prev_obstacle_waypoint;
      displayObstacle(EControl::OTHERS, obstacle_points, obstacle_pub);
      return prev_detection;
    }
  }

  // there are no obstacles, so we move forward
  *obstacle_waypoint = -1;
  false_count = 0;
  prev_detection = EControl::KEEP;
  return detection_result;
}

// detect location state
EControl locationDetection(const VelocitySetInfo& vs_info, EControl& location_result) {
  // 如果ndt_matching中的匹配分数小于阈值，就返回KEEP状态
  if (vs_info.getFitnessScore() <= vs_info.getPoseMatchThreshold()) {
    // std::cout<<"fitness score: "<<fitness_score<<std::endl;
    return location_result;
  }

  location_result = EControl::STOP;
  return location_result;
}

// 根据 detection_result（障碍物检测结果）,去修改 final_waypoints（如果有障碍物，就停避，即插入一段停车轨迹点）
void changeWaypoints(const VelocitySetInfo& vs_info, const EControl& detection_result, int closest_waypoint,
                     int obstacle_waypoint, const ros::Publisher& final_waypoints_pub, VelocitySetPath* vs_path)
{
  if (detection_result == EControl::STOP || detection_result == EControl::STOPLINE)
  {  // STOP for obstacle/stopline
    // stop_waypoint is about stop_distance meter away from obstacles/stoplines
    int stop_distance = (detection_result == EControl::STOP)
      ? vs_info.getStopDistanceObstacle() : vs_info.getStopDistanceStopline();  // 得到 “距离Stopline或Obstacle 多少距离 停车”。这是launch里载入的
    double deceleration = (detection_result == EControl::STOP)
      ? vs_info.getDecelerationObstacle() : vs_info.getDecelerationStopline();  // 得到 “ 上述停车过程中 的加速度”。也是launch里载入的
    // 根据 障碍物位置，和停车距离，计算出停车点
    int stop_waypoint =
        calcWaypointIndexReverse(vs_path->getPrevWaypoints(), obstacle_waypoint, stop_distance);
    // change waypoints to stop by the stop_waypoint
    // changeWaypointsForStopping() 在 当前位置(closest_waypoint) 和 障碍物位置(obstacle_waypoint) 间 设置一条 减速至停车 的轨迹
    vs_path->changeWaypointsForStopping(stop_waypoint, obstacle_waypoint, closest_waypoint, deceleration);
    // avoidSuddenAcceleration() 检查changeWaypointsForStopping()得到的停避轨迹，是否速度上有大的变化，如果有，就进行平滑处理
    vs_path->avoidSuddenAcceleration(deceleration, closest_waypoint);
    vs_path->avoidSuddenDeceleration(vs_info.getVelocityChangeLimit(), deceleration, closest_waypoint);  // 同理，限制减速度，作平滑
    // 从 new_waypoints_ 中获取 temporal_waypoints_size 长度 的一段轨迹（从closest_waypoint开始）
    vs_path->setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), closest_waypoint, vs_info.getControlPose());  // vs_info.getControlPose()是返回base_link的pose
    // final_waypoints_pub 将 /final_waypoints（包含位置和速度） 发布出去，提供给控制模块
    final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
  else if (detection_result == EControl::DECELERATE)
  {  // DECELERATE for obstacles
    vs_path->initializeNewWaypoints();
    vs_path->changeWaypointsForDeceleration(vs_info.getDecelerationObstacle(), closest_waypoint, obstacle_waypoint);
    vs_path->avoidSuddenDeceleration(vs_info.getVelocityChangeLimit(), vs_info.getDecelerationObstacle(), closest_waypoint);
    vs_path->avoidSuddenAcceleration(vs_info.getDecelerationObstacle(), closest_waypoint);
    vs_path->setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), closest_waypoint, vs_info.getControlPose());
    final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
  else
  {  // ACCELERATE or KEEP
    vs_path->initializeNewWaypoints();
    vs_path->avoidSuddenAcceleration(vs_info.getDecelerationObstacle(), closest_waypoint);
    vs_path->avoidSuddenDeceleration(vs_info.getVelocityChangeLimit(), vs_info.getDecelerationObstacle(), closest_waypoint);
    vs_path->setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), closest_waypoint, vs_info.getControlPose());
    final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
}

}  // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_set");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  bool use_crosswalk_detection;
  bool enable_multiple_crosswalk_detection;

  std::string points_topic;
  private_nh.param<bool>("use_crosswalk_detection", use_crosswalk_detection, true);
  private_nh.param<bool>("enable_multiple_crosswalk_detection", enable_multiple_crosswalk_detection, true);

  // 用哪个点云话题，当作STOPLINE的检测
  private_nh.param<std::string>("points_topic", points_topic, "points_lanes");

  // class
  CrossWalk crosswalk;
  VelocitySetPath vs_path;
  VelocitySetInfo vs_info;

  // velocity set subscriber
  // /safety_waypoints是astar_avoid局部规划的输出
  ros::Subscriber waypoints_sub = nh.subscribe("safety_waypoints", 1, &VelocitySetPath::waypointsCallback, &vs_path);
  ros::Subscriber current_vel_sub =
      nh.subscribe("current_velocity", 1, &VelocitySetPath::currentVelocityCallback, &vs_path);

  // 订阅场景管理命令，用于限制本节点只能在某些场景下运行
  ros::Subscriber sub_scenario = nh.subscribe("scenario_manager/scenario_cmd",1,&VelocitySetPath::callbackGetScenario, &vs_path);



  // velocity set info subscriber
  ros::Subscriber config_sub = nh.subscribe("config/velocity_set", 1, &VelocitySetInfo::configCallback, &vs_info);
  ros::Subscriber points_sub = nh.subscribe(points_topic, 1, &VelocitySetInfo::pointsCallback, &vs_info);
  ros::Subscriber localizer_sub = nh.subscribe("localizer_pose", 1, &VelocitySetInfo::localizerPoseCallback, &vs_info);
  ros::Subscriber control_pose_sub = nh.subscribe("current_pose", 1, &VelocitySetInfo::controlPoseCallback, &vs_info);
  ros::Subscriber detectionresult_sub = nh.subscribe("/state/stopline_wpidx", 1, &VelocitySetInfo::detectionCallback, &vs_info);
  ros::Subscriber location_sub = nh.subscribe("/ndt_stat", 1, &VelocitySetInfo::locationCallback, &vs_info);

  // vector map subscriber
  ros::Subscriber sub_dtlane = nh.subscribe("vector_map_info/cross_walk", 1, &CrossWalk::crossWalkCallback, &crosswalk);
  ros::Subscriber sub_area = nh.subscribe("vector_map_info/area", 1, &CrossWalk::areaCallback, &crosswalk);
  ros::Subscriber sub_line = nh.subscribe("vector_map_info/line", 1, &CrossWalk::lineCallback, &crosswalk);
  ros::Subscriber sub_point = nh.subscribe("vector_map_info/point", 1, &CrossWalk::pointCallback, &crosswalk);

  // publisher
  ros::Publisher detection_range_pub = nh.advertise<visualization_msgs::MarkerArray>("detection_range", 1);
  ros::Publisher obstacle_pub = nh.advertise<visualization_msgs::Marker>("obstacle", 1);
  ros::Publisher obstacle_waypoint_pub = nh.advertise<std_msgs::Int32>("obstacle_waypoint", 1, true);
  ros::Publisher stopline_waypoint_pub = nh.advertise<std_msgs::Int32>("stopline_waypoint", 1, true);

  // final_waypoints publisher
  ros::Publisher final_waypoints_pub;
  final_waypoints_pub = nh.advertise<autoware_msgs::Lane>("final_waypoints", 1, true);

  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok())
  {
    ros::spinOnce();

    // 只有在场景为停车场景(使用A*场景)时，才会激活velocity_set节点
    if (vs_path.getActivateScenario()) {
      
      int closest_waypoint = 0;

      if (crosswalk.loaded_all && !crosswalk.set_points)
        crosswalk.setCrossWalkPoints();

      if (!vs_info.getSetPose() || !vs_path.getSetPath() || vs_path.getPrevWaypointsSize() == 0)
      {
        loop_rate.sleep();
        continue;
      }

      crosswalk.setMultipleDetectionFlag(enable_multiple_crosswalk_detection);

      if (use_crosswalk_detection)
        crosswalk.setDetectionWaypoint(
            crosswalk.findClosestCrosswalk(closest_waypoint, vs_path.getPrevWaypoints(), STOP_SEARCH_DISTANCE));

      // detection_result 返回 状态机 的状态，有三种状态：KEEP, STOP, DECELERATE
      int obstacle_waypoint = -1;
      EControl detection_result = obstacleDetection(closest_waypoint, vs_path.getPrevWaypoints(), crosswalk, vs_info,
                                                    detection_range_pub, obstacle_pub, &obstacle_waypoint);
      // 额外 增加一个 定位状态判断
      detection_result = locationDetection(vs_info, detection_result);

      // /final_waypoints这个话题用于给到控制节点，要限制op planner中的final_waypoints和velocity_set中的final_waypoints，不能同时发布
      // 根据障碍物检测结果反馈的 状态机状态，相应地修改final_waypoints（如果有障碍物，就停避，即插入一段停车轨迹点）
      changeWaypoints(vs_info, detection_result, closest_waypoint,
                      obstacle_waypoint, final_waypoints_pub, &vs_path);

      vs_info.clearPoints();

      // publish obstacle waypoint index
      std_msgs::Int32 obstacle_waypoint_index;
      std_msgs::Int32 stopline_waypoint_index;
      if (detection_result == EControl::STOP)
      {
        obstacle_waypoint_index.data = obstacle_waypoint;
        stopline_waypoint_index.data = -1;
      }
      else if (detection_result == EControl::STOPLINE)
      {
        obstacle_waypoint_index.data = -1;
        stopline_waypoint_index.data = obstacle_waypoint;
      }
      else
      {
        obstacle_waypoint_index.data = -1;
        stopline_waypoint_index.data = -1;
      }
      obstacle_waypoint_pub.publish(obstacle_waypoint_index);
      stopline_waypoint_pub.publish(stopline_waypoint_index);

      vs_path.resetFlag();
    }

    loop_rate.sleep();
  }

  return 0;
}
