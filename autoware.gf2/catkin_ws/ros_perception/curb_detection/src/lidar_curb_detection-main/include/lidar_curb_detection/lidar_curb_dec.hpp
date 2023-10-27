/*
 * @Description: 
 * @Author: speedzjy
 * @Date: 2022-04-27 11:49:31
 */

#pragma once

#ifndef LIDAR_CURB_DEC_H
#define LIDAR_CURB_DEC_H

#include "lidar_curb_detection/curb_utils.hpp"
#include "lidar_curb_detection/cloud_mapper.hpp"
#include "lidar_curb_detection/ground_segment.hpp"
#include "lidar_curb_detection/feature_points.hpp"
#include "lidar_curb_detection/boundary_points.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>





namespace CurbDectection {

class LidarCurbDectection : public ParamServer {

public:
  LidarCurbDectection();
  ~LidarCurbDectection();

 // void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr);
  void Syncallback1(const sensor_msgs::PointCloud2ConstPtr& _cloud_ptr1,
  const sensor_msgs::PointCloud2ConstPtr& _cloud_ptr2,
  const sensor_msgs::PointCloud2ConstPtr& _cloud_ptr3);


  // 输入的每帧点云
  PointCloudType::Ptr complete_points;
  CloudQueue queue_complete_points;
  CloudQueue queue_complete_points2;
  CloudQueue queue_complete_points3;

  PointCloudType::Ptr ground_points1;
  PointCloudType::Ptr no_ground_points1;
  PointCloudType::Ptr curb_left  ;
  PointCloudType::Ptr curb_right  ;

  // 边界点云
  CloudPtrList boundary_points;

private:
  ros::Subscriber subPointCloud_;
  ros::Subscriber subGround_PointCloud_;
  ros::Subscriber subNO_ground_PointCloud_;



  ros::Publisher pubCompleteCloud_;
  ros::Publisher pubGroundCloud_;
  ros::Publisher pubNoGroundCloud_;
  ros::Publisher pubFeatureCloud_;
  ros::Publisher pubCurbCloudLeft_;
  ros::Publisher pubCurbCloudRight_;

  ros::Publisher pubMarker_;
  ros::Publisher pubMarker_Bline_;

};

}

#endif