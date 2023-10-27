/*
* Copyright 2021. Perception Engine Inc. All rights reserved.
*
*/

#include <string>
#include <vector>

#include <ros/ros.h>
#include <can_msgs/Frame.h>

#include "ars408_ros/ars408_driver.h"

class PeContinentalArs408Status
{
  ros::NodeHandle global_node_handle_,
    private_node_handle_;
  ros::Subscriber subscriber_can_raw_;
  ros::Publisher publisher_can_setup_;

  ars408::Ars408Driver ars408_driver_;
  ars408::RadarState ars408_state_;

  bool perform_setup_;

  void PerformSetup(const can_msgs::Frame::ConstPtr& can_msg)
  {
    ROS_INFO_STREAM("Generating CAN message to execute setup...");
    if(ars408_state_.OutputType == ars408::RadarState::OBJECTS)
    {
      ROS_WARN_STREAM("Not configuring. Sensor is set to OBJECTS Mode.");
      ros::shutdown();
    }
    else
    {
      ars408::RadarCfg radar_new_config;
      radar_new_config.OutputType = ars408::RadarCfg::OBJECTS;
      radar_new_config.UpdateOutputType = true;

      radar_new_config.StoreInNVM = true;
      radar_new_config.UpdateStoreInNVM = true;

      radar_new_config.SendQuality = true;
      radar_new_config.UpdateSendQuality = true;

      radar_new_config.SendExtInfo = true;
      radar_new_config.UpdateSendExtInfo = true;

      radar_new_config.RadarPower = ars408::RadarCfg::RadarPowerConfig::MINUS_9dB_GAIN;
      radar_new_config.UpdateRadarPower = true;

      radar_new_config.RCS_Status = ars408::RadarCfg::RCS_Threshold::HIGH_SENSITIVITY;
      radar_new_config.UpdateRCS_Threshold = true;

      radar_new_config.SortIndex = ars408::RadarCfg::Sorting::BY_RANGE;
      radar_new_config.UpdateSortIndex = true;

      boost::array<uint8_t, 8> can_command;
      can_command = ars408_driver_.GenerateRadarConfiguration(radar_new_config);

      can_msgs::Frame config_can_frame;
      config_can_frame.header = can_msg->header;
      config_can_frame.data = can_command;
      config_can_frame.id = ars408::RADAR_CFG;
      config_can_frame.dlc = ars408::RADAR_CFG_BYTES;
      ROS_INFO_STREAM("Configuration Message Generated. Publishing.");
      publisher_can_setup_.publish(config_can_frame);
    }
  }

  void CanFrameCallback(const can_msgs::Frame::ConstPtr& can_msg)
  {
    if (!can_msg->data.empty())
    {
      ars408_driver_.Parse(can_msg->id, can_msg->data, can_msg->dlc);

      if(ars408_driver_.GetCurrentRadarState(ars408_state_))
      {
        ROS_INFO_STREAM(ars408_state_.ToString());
        ROS_INFO_STREAM("Status Obtained.");
        if(perform_setup_)
        {
          PerformSetup(can_msg);
          ROS_INFO_STREAM("Waiting for sensor. Please wait...");
          ros::Duration(10.0).sleep();
        }
      }
      else
      {
        ROS_INFO_STREAM_THROTTLE(0.5, "Waiting for valid status command...");
      }
    }
  }

public:
  PeContinentalArs408Status() : global_node_handle_(), private_node_handle_("~"), ars408_driver_()
  {
  }

  void Run()
  {
    std::string can_input_topic, can_output_topic;

    private_node_handle_.param<std::string>("can_input_topic",
                                            can_input_topic,
                                            "can_raw");

    private_node_handle_.param<std::string>("can_output_topic",
                                            can_output_topic,
                                            "can_to_device");

    private_node_handle_.param<bool>("perform_setup",
                                            perform_setup_,
                                     false);

    subscriber_can_raw_ = global_node_handle_.subscribe(can_input_topic,
                                                        10,
                                                        &PeContinentalArs408Status::CanFrameCallback,
                                                        this);

    publisher_can_setup_ = global_node_handle_.advertise<can_msgs::Frame>(can_output_topic,
                                                                          10,
                                                                          false);

    ROS_INFO_STREAM("Subscribed to " << can_input_topic);
    ROS_INFO_STREAM("Waiting for RADAR_STATE(0x201) message...");

    ros::spin();
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pe_ars408_ros_status");

  PeContinentalArs408Status app;

  app.Run();

  return 0;
}
