/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
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
 * 
 * 
 * Mark Jin edit to addapt autoware.ai with robobus firmware. 20211227.
 */

#ifndef VEHICLE_RAW_VEHICLE_CMD_CONVERTER_INCLUDE_RAW_VEHICLE_CMD_CONVERTER_NODE_HPP_
#define VEHICLE_RAW_VEHICLE_CMD_CONVERTER_INCLUDE_RAW_VEHICLE_CMD_CONVERTER_NODE_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <autoware_vehicle_msgs/RawVehicleCommand.h>
#include <autoware_vehicle_msgs/Shift.h>
#include <autoware_msgs/VehicleCmd.h>
#include "pix_driver_msgs/acu_sweepctrlcmd_107.h"

#include <raw_vehicle_cmd_converter/accel_map.h>
#include <raw_vehicle_cmd_converter/brake_map.h>

class AccelMapConverter
{
public:
  AccelMapConverter();
  ~AccelMapConverter() = default;

private:
  ros::NodeHandle nh_;              //!< @brief ros node handle
  ros::NodeHandle pnh_;             //!< @brief private ros node handle
  ros::Publisher pub_cmd_;          //!< @brief topic publisher for low level vehicle command
  ros::Publisher pub_sweeper_cmd_;
  ros::Subscriber sub_velocity_;    //!< @brief subscriber for current velocity
  ros::Subscriber sub_cmd_;         //!< @brief subscriber for vehicle command
  ros::Subscriber sub_sweeper_state_; //!< #brief subscriber for sweeper command 
  ros::Timer timer_;

  std::shared_ptr<double> current_velocity_ptr_;  // [m/s]
  autoware_vehicle_msgs::RawVehicleCommand output;

  AccelMap accel_map_;
  BrakeMap brake_map_;
  bool acc_map_initialized_;
  double max_throttle_;
  double max_brake_;
  bool sweeper_status_ = false;
  pix_driver_msgs::acu_sweepctrlcmd_107 acu_sweepctrlcmd_107;

  void callbackVehicleCmd(const autoware_msgs::VehicleCmdConstPtr vehicle_cmd_ptr);
  void callbackVelocity(const geometry_msgs::TwistStampedConstPtr msg);
  void callbackSweeperCmd(const std_msgs::Bool &msg);
  void calculateAccelMap(
    const double current_velocity, const double desired_acc, double * desired_throttle,
    double * desired_brake);
  void timer_callback(const ros::TimerEvent &te);
};

#endif  // VEHICLE_RAW_VEHICLE_CMD_CONVERTER_INCLUDE_RAW_VEHICLE_CMD_CONVERTER_NODE_HPP_