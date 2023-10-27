/*
* Copyright 2021. Perception Engine Inc. All rights reserved.
*
*/
#ifndef ARS408_ROS_H
#define ARS408_ROS_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <unique_id/unique_id.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "ars408_ros/ars408_driver.h"
#include "autoware_perception_msgs/DynamicObjectArray.h"

class PeContinentalArs408Node
{
  ros::NodeHandle global_node_handle_,
                private_node_handle_;
  ros::Subscriber subscriber_can_raw_;
  ros::Publisher publisher_dynamic_object_array_;
  ros::Timer diagnostic_timer_;

  ros::Time last_subscribed_time_;

  diagnostic_updater::Updater diagnostic_updater_;

  std::string output_frame_;
  uint32_t sequence_id_;

  uint16_t max_radar_id = 512;

  bool ready_;

  std::vector<uuid_msgs::UniqueID> uuid_array_;

  ars408::Ars408Driver ars408_driver_;
  ars408::RadarState ars408_state_;

  void CanFrameCallback(const can_msgs::Frame::ConstPtr& can_msg);
  void DiagnosticsTimer(const ros::TimerEvent& event);

  autoware_perception_msgs::DynamicObject
  ConvertRadarObjectToAwDynamicObject(const ars408::RadarObject& in_object);

  static uint32_t
  ConvertRadarClassToAwSemanticClass(const ars408::Obj_3_Extended::ObjectClassProperty& in_radar_class);

  void GenerateUuidTable();

  void CheckPersistentError(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void CheckTemporaryError(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void CheckTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void CheckVoltage(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void CheckConnection(diagnostic_updater::DiagnosticStatusWrapper& stat);
public:
  PeContinentalArs408Node();
  void RadarDetectedObjectsCallback(const std::unordered_map<uint8_t , ars408::RadarObject>& detected_objects);
  void Run();
};

#endif //ARS408_ROS_H