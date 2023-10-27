/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
 *  Mark Jin edit to addapt autoware.ai with robobus firmware. 20211227.
 */

#include "raw_vehicle_cmd_converter/node.hpp"

AccelMapConverter::AccelMapConverter() : nh_(""), pnh_("~")
{
  pub_cmd_ = nh_.advertise<autoware_vehicle_msgs::RawVehicleCommand>("/vehicle/raw_vehicle_cmd", 1);
  pub_sweeper_cmd_ = nh_.advertise<pix_driver_msgs::acu_sweepctrlcmd_107>("/pix/acu_sweepctrlcmd", 1);
  sub_cmd_ = nh_.subscribe("/vehicle_cmd", 1, &AccelMapConverter::callbackVehicleCmd, this);
  sub_sweeper_state_ = nh_.subscribe("/sweeper_status", 1, &AccelMapConverter::callbackSweeperCmd, this);
  sub_velocity_ =
    nh_.subscribe("/current_velocity", 1, &AccelMapConverter::callbackVelocity, this);

  timer_ = nh_.createTimer(ros::Duration(0.01), &AccelMapConverter::timer_callback, this);

  pnh_.param<double>("max_throttle", max_throttle_, 0.2);
  pnh_.param<double>("max_brake", max_brake_, 0.8);

  /* parameters for accel/brake map */
  std::string csv_path_accel_map, csv_path_brake_map;
  pnh_.param<std::string>("csv_path_accel_map", csv_path_accel_map, std::string("empty"));
  pnh_.param<std::string>("csv_path_brake_map", csv_path_brake_map, std::string("empty"));
  acc_map_initialized_ = true;
  if (!accel_map_.readAccelMapFromCSV(csv_path_accel_map)) {
    ROS_ERROR("Cannot read accelmap. csv path = %s. stop calculation.", csv_path_accel_map.c_str());
    acc_map_initialized_ = false;
  }
  if (!brake_map_.readBrakeMapFromCSV(csv_path_brake_map)) {
    ROS_ERROR("Cannot read brakemap. csv path = %s. stop calculation.", csv_path_brake_map.c_str());
    acc_map_initialized_ = false;
  }
  acu_sweepctrlcmd_107.AutoCleaningStartCtrl = 0;
}

void AccelMapConverter::callbackVelocity(const geometry_msgs::TwistStampedConstPtr msg)
{
  current_velocity_ptr_ = std::make_shared<double>(msg->twist.linear.x);
}

void AccelMapConverter::callbackVehicleCmd(
  const autoware_msgs::VehicleCmdConstPtr vehicle_cmd_ptr)
{
  if (!current_velocity_ptr_ || !acc_map_initialized_) {
    return;
  }

  double desired_throttle = 0.0;
  double desired_brake = 0.0;
  double linear_acceleration = vehicle_cmd_ptr->ctrl_cmd.linear_acceleration;
  if(vehicle_cmd_ptr->gear_cmd.gear == 2){
    max_throttle_ = 0.05;
    linear_acceleration = abs(linear_acceleration);
  }
  calculateAccelMap(
    *current_velocity_ptr_, linear_acceleration, 
    &desired_throttle, &desired_brake);
  // autoware_vehicle_msgs::RawVehicleCommand output;
  output.header = vehicle_cmd_ptr->header;
  output.shift.data = vehicle_cmd_ptr->gear_cmd.gear;
  
  output.emergency = vehicle_cmd_ptr->emergency;
  output.control.steering_angle = vehicle_cmd_ptr->ctrl_cmd.steering_angle*200;
  output.control.steering_angle_velocity = 0.0;
  output.control.throttle = desired_throttle;
  output.control.brake = desired_brake;

  // pub_cmd_.publish(output);
}

void AccelMapConverter::calculateAccelMap(
  const double current_velocity, const double desired_acc, double * desired_throttle,
  double * desired_brake)
{
  // throttle mode
  if (!accel_map_.getThrottle(desired_acc, std::abs(current_velocity), *desired_throttle)) {
    // brake mode
    *desired_throttle = 0.0;
    brake_map_.getBrake(desired_acc, std::abs(current_velocity), *desired_brake);
  }
  *desired_throttle = std::min(std::max(*desired_throttle, 0.0), max_throttle_);
  *desired_brake = std::min(std::max(*desired_brake, 0.0), max_brake_);
}

void AccelMapConverter::timer_callback(const ros::TimerEvent &te){
  pub_cmd_.publish(output);
  pub_sweeper_cmd_.publish(acu_sweepctrlcmd_107);
}

void AccelMapConverter::callbackSweeperCmd(const std_msgs::Bool &msg){
  sweeper_status_ = msg.data;
  if(sweeper_status_){
    acu_sweepctrlcmd_107.AutoCleaningStartCtrl = 1;
  }else{
    acu_sweepctrlcmd_107.AutoCleaningStartCtrl = 2;
  }
}