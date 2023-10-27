/*
 * Copyright 2021. Perception Engine Inc. All rights reserved.
 */
#include "ars408_ros/ars408_ros_node.h"


PeContinentalArs408Node::PeContinentalArs408Node() :
  global_node_handle_(),
  private_node_handle_("~"),
  ars408_driver_(),
  sequence_id_(0),
  last_subscribed_time_(ros::Time::now()),
  ready_(false)
{
  GenerateUuidTable();
}

void PeContinentalArs408Node::CanFrameCallback(const can_msgs::Frame::ConstPtr& can_msg)
{
  if (!can_msg->data.empty())
  {
    ars408_driver_.Parse(can_msg->id, can_msg->data, can_msg->dlc);
    last_subscribed_time_ = can_msg->header.stamp;
    ready_ = true;
  }
}

void PeContinentalArs408Node::DiagnosticsTimer(const ros::TimerEvent& event)
{
  if(ars408_driver_.GetCurrentRadarState(ars408_state_) && ready_)
  {
    diagnostic_updater_.force_update();
    ready_ = false;
  }
}

void PeContinentalArs408Node::GenerateUuidTable()
{
  for(size_t i=0; i < max_radar_id; i++)
  {
    uuid_array_.emplace_back(unique_id::toMsg(unique_id::fromRandom()));
  }
  ROS_INFO("Created: %ld/%ld Ids", max_radar_id, uuid_array_.size());
}

void PeContinentalArs408Node::CheckPersistentError(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (ars408_state_.PersistentError) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "Persistent Error";
  }

  stat.summary(level, msg);
}

void PeContinentalArs408Node::CheckTemporaryError(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (ars408_state_.TemporaryError) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "Temporary Error";
  }

  stat.summary(level, msg);
}

void PeContinentalArs408Node::CheckTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (ars408_state_.TemperatureError) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "Temperature Error";
  }

  stat.summary(level, msg);
}

void PeContinentalArs408Node::CheckVoltage(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (ars408_state_.VoltageError) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "Voltage Error";
  }

  stat.summary(level, msg);
}

void PeContinentalArs408Node::CheckConnection(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (ros::Time::now() - last_subscribed_time_ > ros::Duration(1.0)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "Connection Timeout";
  }

  stat.summary(level, msg);
}

uint32_t
PeContinentalArs408Node::ConvertRadarClassToAwSemanticClass(const ars408::Obj_3_Extended::ObjectClassProperty& in_radar_class)
{
  switch (in_radar_class)
  {
    case ars408::Obj_3_Extended::BICYCLE:
      return autoware_perception_msgs::Semantic::BICYCLE;
      break;
    case ars408::Obj_3_Extended::CAR:
      return autoware_perception_msgs::Semantic::CAR;
      break;
    case ars408::Obj_3_Extended::TRUCK:
      return autoware_perception_msgs::Semantic::TRUCK;
      break;
    case ars408::Obj_3_Extended::MOTORCYCLE:
      return autoware_perception_msgs::Semantic::MOTORBIKE;
      break;
    case ars408::Obj_3_Extended::POINT:
    case ars408::Obj_3_Extended::RESERVED_01:
    case ars408::Obj_3_Extended::WIDE:
    case ars408::Obj_3_Extended::RESERVED_02:
    default:
      return autoware_perception_msgs::Semantic::UNKNOWN;
      break;
  }
}

autoware_perception_msgs::DynamicObject
PeContinentalArs408Node::ConvertRadarObjectToAwDynamicObject(const ars408::RadarObject& in_object)
{
  autoware_perception_msgs::DynamicObject out_object;

  if (in_object.id < max_radar_id)
  {

    out_object.id = uuid_array_[in_object.id];
  }
  else
  {
    out_object.id = unique_id::toMsg(unique_id::fromRandom());
  }
  out_object.semantic.type = ConvertRadarClassToAwSemanticClass(in_object.object_class);
  out_object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  out_object.shape.dimensions.x = 1.0;
  out_object.shape.dimensions.y = 1.0;
  out_object.shape.dimensions.z = 2.0;

  out_object.semantic.confidence = in_object.rcs;
  out_object.state.pose_covariance.pose.position.x = in_object.distance_long_x;
  out_object.state.pose_covariance.pose.position.y = in_object.distance_lat_y;

  out_object.state.twist_reliable = true;
  out_object.state.twist_covariance.twist.linear.x = in_object.speed_long_x;
  out_object.state.twist_covariance.twist.linear.y = in_object.speed_lat_y;
  out_object.state.twist_covariance.twist.angular.x = in_object.speed_long_x;
  out_object.state.twist_covariance.twist.angular.y = in_object.speed_lat_y;

  out_object.state.acceleration_reliable = true;
  out_object.state.acceleration_covariance.accel.angular.x = in_object.rel_acceleration_long_x;
  out_object.state.acceleration_covariance.accel.angular.y = in_object.rel_acceleration_lat_y;

  return out_object;
}

void PeContinentalArs408Node::RadarDetectedObjectsCallback(const std::unordered_map<uint8_t ,
                                                           ars408::RadarObject>& detected_objects)
{
  autoware_perception_msgs::DynamicObjectArray aw_output_objects;

  aw_output_objects.header.frame_id = output_frame_;
  ros::Time current_time = ros::Time::now();
  aw_output_objects.header.stamp = current_time;
  aw_output_objects.header.seq = sequence_id_++;

  for(auto object: detected_objects)
  {
    //ROS_INFO_STREAM(object.second.ToString());
    autoware_perception_msgs::DynamicObject aw_object = ConvertRadarObjectToAwDynamicObject(object.second);
    aw_output_objects.objects.emplace_back(aw_object);
  }
  publisher_dynamic_object_array_.publish(aw_output_objects);
}

void PeContinentalArs408Node::Run()
{
  std::string can_input_topic, object_output_topic;

  private_node_handle_.param<std::string>("can_input_topic",
                                          can_input_topic,
                                          "can_raw");
  private_node_handle_.param<std::string>("object_output_topic",
                                          object_output_topic,
                                          "/detection/radar/objects");
  private_node_handle_.param<std::string>("output_frame",
                                          output_frame_,
                                          "ars408");

  ars408_driver_.RegisterDetectedObjectsCallback(
    boost::bind(&PeContinentalArs408Node::RadarDetectedObjectsCallback,
                this,
                _1));

  subscriber_can_raw_ = global_node_handle_.subscribe(can_input_topic,
                                                      10,
                                                      &PeContinentalArs408Node::CanFrameCallback,
                                                      this);
  ROS_INFO_STREAM("Subscribed to " << can_input_topic);

  publisher_dynamic_object_array_ =
    global_node_handle_.advertise<autoware_perception_msgs::DynamicObjectArray>(object_output_topic,
                                                                                10,
                                                                                true);

  diagnostic_timer_ = global_node_handle_.createTimer(ros::Duration(1.0),
                                                      &PeContinentalArs408Node::DiagnosticsTimer,
                                                      this);

  diagnostic_updater_.setHardwareID("ars408-" + output_frame_);
  diagnostic_updater_.add("radar_persistent_error", this, &PeContinentalArs408Node::CheckPersistentError);
  diagnostic_updater_.add("radar_temporary_error", this, &PeContinentalArs408Node::CheckTemporaryError);
  diagnostic_updater_.add("radar_temperature", this, &PeContinentalArs408Node::CheckTemperature);
  diagnostic_updater_.add("radar_voltage", this, &PeContinentalArs408Node::CheckVoltage);
  diagnostic_updater_.add("radar_connection", this, &PeContinentalArs408Node::CheckConnection);

  ros::spin();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pe_ars408_ros_node");

  PeContinentalArs408Node app;

  app.Run();

  return 0;
}
