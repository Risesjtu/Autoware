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

#include "nmea2tfpose_core.h"

namespace gnss_localizer
{
// Constructor
Nmea2TFPoseNode::Nmea2TFPoseNode()
  : private_nh_("~")
  , MAP_FRAME_("map")
  , GPS_FRAME_("gps")
  , roll_(0)
  , pitch_(0)
  , yaw_(0)
  , orientation_time_(-std::numeric_limits<double>::infinity())
  , position_time_(-std::numeric_limits<double>::infinity())
  , current_time_(0)
  , orientation_stamp_(0)
  , orientation_ready_(false)
{
  initForROS();
  geo_.set_plane(plane_number_);
}

// Destructor
Nmea2TFPoseNode::~Nmea2TFPoseNode()
{
}

void Nmea2TFPoseNode::initForROS()
{
  // ros parameter settings
  private_nh_.getParam("plane", plane_number_);

  // setup subscriber
  sub1_ = nh_.subscribe("nmea_sentence", 100, &Nmea2TFPoseNode::callbackFromNmeaSentence, this);

  std::string fp_llh_topic, fp_ypr_topic;
  private_nh_.getParam("fp_llh_topic", fp_llh_topic);
  private_nh_.getParam("fp_ypr_topic", fp_ypr_topic);
  sub2_ = nh_.subscribe(fp_llh_topic.c_str(), 100, &Nmea2TFPoseNode::callbackFrom_FPNavSatFix, this);
  sub3_ = nh_.subscribe(fp_ypr_topic.c_str(), 100, &Nmea2TFPoseNode::callbackFrom_FPYPR, this);

  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10);
  
  pub2_ = nh_.advertise<std_msgs::Float64MultiArray>("lat_lon", 10);  // khy add 
}

void Nmea2TFPoseNode::run()
{
  ros::spin();
}

void Nmea2TFPoseNode::publishPoseStamped()
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = MAP_FRAME_;
  pose.header.stamp = current_time_;
  pose.pose.position.x = geo_.y();
  pose.pose.position.y = geo_.x();
  pose.pose.position.z = geo_.z();
  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, yaw_);
  pub1_.publish(pose);
}

void Nmea2TFPoseNode::publishTF()
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(geo_.y(), geo_.x(), geo_.z()));
  tf::Quaternion quaternion;
  quaternion.setRPY(roll_, pitch_, yaw_);
  transform.setRotation(quaternion);
  br_.sendTransform(tf::StampedTransform(transform, current_time_, MAP_FRAME_, GPS_FRAME_));
}

void Nmea2TFPoseNode::createOrientation()
{
  yaw_ = atan2(geo_.x() - last_geo_.x(), geo_.y() - last_geo_.y());
  roll_ = 0;
  pitch_ = 0;
}

// int32_t count = 0;  // khy add
void Nmea2TFPoseNode::convert(std::vector<std::string> nmea, ros::Time current_stamp)
{
  try
  {
    // count++;  // khy add
    if (nmea.at(0).compare(0, 2, "QQ") == 0)
    {
      orientation_time_ = stod(nmea.at(3));
      roll_ = stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(6)) * M_PI / 180. + M_PI / 2;
      orientation_stamp_ = current_stamp;
      orientation_ready_ = true;
      ROS_INFO("QQ is subscribed.");
    }
    else if (nmea.at(0) == "$PASHR")
    {
      orientation_time_ = stod(nmea.at(1));
      roll_ = stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(2)) * M_PI / 180. + M_PI / 2;
      orientation_ready_ = true;
      ROS_INFO("PASHR is subscribed.");
    }
    else if (nmea.at(0).compare(3, 3, "GGA") == 0)
    {
      position_time_ = stod(nmea.at(1));
      double lat = stod(nmea.at(2));
      double lon = stod(nmea.at(4));
      double h = stod(nmea.at(9));

      if (nmea.at(3) == "S")
        lat = -lat;

      if (nmea.at(5) == "W")
        lon = -lon;

      geo_.set_llh_nmea_degrees(lat, lon, h);

      ROS_INFO("GGA is subscribed.");
    //   if (count % 50 == 0) {
    //     // khy  add
    //     std_msgs::Float64MultiArray lat_lon;
    //     std::vector<double> lat_lon_;
    //     lat_lon_.push_back(lat / 100.0);  // 除以100是因为lat比正常的纬度多了两位
    //     lat_lon_.push_back(lon / 100.0);
    //     lat_lon.data = lat_lon_;
    //     pub2_.publish(lat_lon);  
    //   }
    }
    else if (nmea.at(0) == "$GPRMC")
    {
      position_time_ = stoi(nmea.at(1));
      double lat = stod(nmea.at(3));
      double lon = stod(nmea.at(5));
      double h = 0.0;

      if (nmea.at(4) == "S")
        lat = -lat;

      if (nmea.at(6) == "W")
        lon = -lon;

      geo_.set_llh_nmea_degrees(lat, lon, h);

      ROS_INFO("GPRMC is subscribed.");
    //   if (count % 50 == 0) {
    //         //   khy  add
    //         // actual:121.443697 31.026718；ref:31°01.72134623' N, 121°26.34220871' E
    //         std_msgs::Float64MultiArray lat_long;
    //         std::vector<double> lat_long_;
    //         lat_long_.push_back(lat / 100.0);  // 除以100是因为lat比正常的纬度多了两位
    //         lat_long_.push_back(lon / 100.0);
    //         lat_long.data = lat_long_;
    //         pub2_.publish(lat_long);  
    //   }
    }
  }
  catch (const std::exception &e)
  {
    ROS_WARN_STREAM("Message is invalid : " << e.what());
  }
}

void Nmea2TFPoseNode::callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg)
{
  current_time_ = msg->header.stamp;
  convert(split(msg->sentence), msg->header.stamp);

  double timeout = 10.0;
  // if orientation_stamp_ is 0 then no "QQ" sentence was ever received,
  // so orientation should be computed from offsets
  if (orientation_stamp_.isZero()
      || fabs(orientation_stamp_.toSec() - msg->header.stamp.toSec()) > timeout)
  {
    double dt = sqrt(pow(geo_.x() - last_geo_.x(), 2) + pow(geo_.y() - last_geo_.y(), 2));
    double threshold = 0.2;
    if (dt > threshold)
    {
      /* If orientation data is not available it is generated based on translation
         from the previous position. For the first message the previous position is
         simply the origin, which gives a wildly incorrect orientation. Some nodes
         (e.g. ndt_matching) rely on that first message to initialise their pose guess,
         and cannot recover from such incorrect orientation.
         Therefore the first message is not published, ensuring that orientation is
         only calculated from sensible positions.
      */
      if (orientation_ready_)
      {
        ROS_INFO("QQ is not subscribed. Orientation is created by atan2");
        createOrientation();
        publishPoseStamped();
        publishTF();
      }
      else
      {
        orientation_ready_ = true;
      }
      last_geo_ = geo_;
    }
    return;
  }

  double e = 1e-2;
  if ((fabs(orientation_time_ - position_time_) < e) && orientation_ready_)
  {
    publishPoseStamped();
    publishTF();
    return;
  }
}

std::vector<std::string> split(const std::string &string)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);

  while (getline(ss, token, ','))
    str_vec_ptr.push_back(token);

  return str_vec_ptr;
}

// header: 
//   seq: 3456
//   stamp: 1679118789.264000000
//   frame_id: POI
// status: 
//   status: 0
//   service: 0
// latitude: 31.027299158000001711
// longitude: 121.43674854500000038
// altitude: 15.856999999999999318
// position_covariance[]
//   position_covariance[0]: 0.17105999999999998984
//   position_covariance[1]: -0.015308999999999999692
//   position_covariance[2]: -0.024018000000000001154
//   position_covariance[3]: -0.015308999999999999692
//   position_covariance[4]: 0.14396999999999998687
//   position_covariance[5]: -0.030394000000000000877
//   position_covariance[6]: -0.024018000000000001154
//   position_covariance[7]: -0.030394000000000000877
//   position_covariance[8]: 0.1038099999999999995
// position_covariance_type: 3
int32_t count = 0;  // khy add
void Nmea2TFPoseNode::callbackFrom_FPNavSatFix(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
  count++;  // khy add
  ROS_DEBUG("subcribe to /fixposition/navsatfix");
  if(msg->status.status == -1) //sensor_msgs::NavSatFix::STATUS_NO_FIX
  {
    ROS_WARN("unable to fix position");
    return;
  }

  // std::cout << std::setprecision(20) << *msg << std::endl << std::endl;

  double lat_deg, lon_deg;
  if (msg->latitude > 0) {
    lat_deg = floor(msg->latitude);
  } else {
    lat_deg = ceil(msg->latitude);
  }

  if(msg->longitude>0)
  {
    lon_deg = floor(msg->longitude);
  } else {
    lon_deg = ceil(msg->longitude);
  }

  if (count % 50 == 0) {
        // khy  add
        std_msgs::Float64MultiArray lat_lon;
        std::vector<double> lat_lon_;
        lat_lon_.push_back(msg->latitude);  // 除以100是因为lat比正常的纬度多了两位
        lat_lon_.push_back(msg->longitude);
        lat_lon.data = lat_lon_;
        pub2_.publish(lat_lon);  
  }

  // double lat, lon;
  // lat = lat_deg*100+(msg->latitude-lat_deg)*60;
  // lon = lon_deg*100+(msg->longitude-lon_deg)*60;
  // std::cout << std::setprecision(20) << lat << " " << lon << std::endl;
  geo_.set_llh_nmea_degrees(lat_deg*100+(msg->latitude-lat_deg)*60, lon_deg*100+(msg->longitude-lon_deg)*60, msg->altitude);
  
  // static double last_x, last_y;
  // double dt = sqrt(pow(geo_.x() - last_x, 2) + pow(geo_.y() - last_y, 2));
  // double threshold = 0.2;
  // if (dt > threshold){
  //   createOrientation();
  //   // std::cout << std::setprecision(20) << "YPR: " << yaw_ << " " << pitch_ << " " << roll_ << std::endl;
  //   last_x = geo_.x();
  //   last_y = geo_.y();
  // }
  publishPoseStamped();
  publishTF();


  last_geo_ = geo_;
}

// /fixposition/ypr
void Nmea2TFPoseNode::callbackFrom_FPYPR(const geometry_msgs::Vector3 &msg)
{
  yaw_ = msg.x;
  pitch_ = msg.y;
  roll_ = msg.z;
}

}  // namespace gnss_localizer



