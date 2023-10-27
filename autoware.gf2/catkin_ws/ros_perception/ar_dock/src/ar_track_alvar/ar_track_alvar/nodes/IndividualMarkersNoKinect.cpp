/*
 Software License Agreement (BSD License)

 Copyright (c) 2012, Scott Niekum
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the Willow Garage nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 author: Scott Niekum
*/


#include "std_msgs/Header.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/Shared.h"
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <autoware_msgs/DockInfo.h>
#include <autoware_msgs/VehicleCleaningStatus.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <ar_track_alvar/ParamsConfig.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"

using namespace alvar;
using namespace std;

// 状态机信息
enum DockState {
  NeedCharge = 1,   // 需要充电, 寻找充电桩位置vector中距离当前车辆最近的充电桩的粗位置, 发布后开始扫描ar码, 并发布精细感知坐标
  Charging = 2,     // 正在充电, 停止扫描
  OffCharge = 3,     // 停止充电, 可能是冗余的
  NeedTrash = 4
};

// 一些需要手动设置(launch设置)的参数变量
bool debug_mode;   // debug_mode=true下, 不需要通过发布充电倒垃圾命令, 直接发送2个初始位姿给规划, 用于longma出差流程
int charge_id, trash_id; // default marker
int marker_resolution = 5; // default marker resolution
int marker_margin = 2; // default marker margin
double dist_before_charge, dist_before_trash;

// 一些订阅、发布的变量
Camera *cam;
cv_bridge::CvImagePtr cv_ptr_;
image_transport::Subscriber cam_sub_;
ros::Publisher arImagePub_;
ros::Publisher arMarkerPub_;
ros::Publisher rvizMarkerPub_;
ros::Publisher arMarkerPubFine_;
ros::Publisher vehicle_status_pub_;
ar_track_alvar_msgs::AlvarMarkers arPoseMarkers_;
visualization_msgs::Marker rvizMarker_;
tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;
MarkerDetector<MarkerData> marker_detector;

// 一些不需要手动设置的参数变量
bool init=true;
bool enableSwitched = false;
bool is_visualization = true;
bool enabled = true;
double max_frequency;
double marker_size;
double max_new_marker_error;
double max_track_error;
std::string cam_image_topic;
std::string cam_info_topic;
std::string output_frame;
double last_coarse_detect_time = 0.0;
bool stop_for_fineture = false;
geometry_msgs::PoseStamped coarse_ar_pose;
geometry_msgs::PoseStamped last_ar_pose;  // 上一次定位的位置
geometry_msgs::PoseStamped current_pose_global;  // 当前车辆在map坐标系下的位置
geometry_msgs::PoseStamped coarse_pose;   // 需要手动测量的粗略位置
DockState charge_status = OffCharge;
autoware_msgs::DrivingStatus end_point_status;

const std::vector<std::vector<double> > CoarseChargingPositions{
  // {-2.7, -5.1, 0, 0, 0, 0.785, 0.619},   // 文治大道, 右边的车位
  {13.0, 24.0, 1, 0, 0, 1, 0},   // 在仿真环境中的真实位置是(24.2946 -12.8606 1.13012 -1.5708 0.0 -0.0), 这里增加误差模拟粗位置, 并且gazebo中的坐标系与rviz即我们的map坐标系坐标轴定义不同
  // {-2.2, -16.4, 1, 0, 0, 0.707, 0.707}
};

const std::vector<std::vector<double> > CoarseTrashbinPositions{
  // {-1.263, -4.793, 1, 0, 0, 0.820, 0.572},  // 文治大道
  {13.0, 24.0, 1, 0, 0, 1, 0},   // 在仿真环境中的真实位置是(24.2946 -12.8606 1.13012 -1.5708 0.0 -0.0), 这里增加误差模拟粗位置, 并且gazebo中的坐标系与rviz即我们的map坐标系坐标轴定义不同
  // {-2.2, -16.4, 1, 0, 0, 0.707, 0.707}
};

void visualizeMarker(geometry_msgs::Pose pose, std_msgs::Header header)
{
  rvizMarker_.header.frame_id = header.frame_id;
  rvizMarker_.header.stamp = header.stamp;

  if (charge_status == NeedCharge)
    rvizMarker_.id = charge_id;
  if (charge_status == NeedTrash)
    rvizMarker_.id = trash_id;

  rvizMarker_.pose = pose;

  rvizMarker_.scale.x = 1.0 * marker_size/100.0;
  rvizMarker_.scale.y = 0.2 * marker_size/100.0;
  rvizMarker_.scale.z = 1.0 * marker_size/100.0;
  rvizMarker_.ns = "basic_shapes";
  rvizMarker_.type = visualization_msgs::Marker::CUBE;
  rvizMarker_.action = visualization_msgs::Marker::ADD;

  rvizMarker_.color.r = 0.5f;
  rvizMarker_.color.g = 0.0f;
  rvizMarker_.color.b = 0.5f;
  rvizMarker_.color.a = 1.0;

  rvizMarker_.lifetime = ros::Duration (1.0);
  rvizMarkerPub_.publish (rvizMarker_);
}

void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg)
{
  if (charge_status != NeedCharge && charge_status != NeedTrash) {   // 都不等于才返回, 有一个等于则false, 不return
    return;  // 如果不是需要充电或倒垃圾状态, 返回, 节省计算量
  }
  
	if(cam->getCamInfo_){
		try{
			tf::StampedTransform CamToOutput;
    			try{
					tf_listener->waitForTransform(output_frame, image_msg->header.frame_id, image_msg->header.stamp, ros::Duration(1.0));
					tf_listener->lookupTransform(output_frame, image_msg->header.frame_id, image_msg->header.stamp, CamToOutput);
   				}
    			catch (tf::TransformException ex){
      				ROS_ERROR("%s",ex.what());
    			}


            //Convert the image
            cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

            //Get the estimated pose of the main markers by using all the markers in each bundle

            // GetMultiMarkersPoses expects an IplImage*, but as of ros groovy, cv_bridge gives
            // us a cv::Mat. I'm too lazy to change to cv::Mat throughout right now, so I
            // do this conversion here -jbinney
            IplImage ipl_image = cv_ptr_->image;
            cv::Mat mat_image_ori = cv_ptr_->image;  // 用于可视化

            marker_detector.Detect(&ipl_image, cam, true, false, max_new_marker_error, max_track_error, CVSEQ, true);
            arPoseMarkers_.markers.clear ();
			for (size_t i=0; i<marker_detector.markers->size(); i++)
			{
				//Get the pose relative to the camera
        int id = (*(marker_detector.markers))[i].GetId();
        // std::cout << id << "\n";
        if (id != charge_id && id != trash_id)
          continue;   // 似乎是纯白或纯黑, 经常在仿真中混淆, 去掉不使用
        printf("marker_%d detected!!\n", id);
				Pose p = (*(marker_detector.markers))[i].pose;
				double px = p.translation[0]/100.0;
				double py = p.translation[1]/100.0;
				double pz = p.translation[2]/100.0;
        double qw = p.quaternion[0];
				double qx = p.quaternion[1];
				double qy = p.quaternion[2];
				double qz = p.quaternion[3];
			
        tf::Quaternion rotation (qx, qy, qz, qw);
        tf::Vector3 origin (px, py, pz);
        tf::Transform t (rotation, origin);
        tf::Vector3 markerOrigin (0, 0, 0);
        tf::Transform m (tf::Quaternion::getIdentity (), markerOrigin);
        tf::Transform markerPose = t * m; // 计算出marker在camera坐标系的的位置

        // tf::Vector3 z_axis_cam = tf::Transform(rotation, tf::Vector3(0, 0, 0)) * tf::Vector3(0, 0, 1);
        // ROS_INFO("%02i Z in cam frame: %f %f %f",id, z_axis_cam.x(), z_axis_cam.y(), z_axis_cam.z());
        /// as we can't see through markers, this one is false positive detection
        // if (z_axis_cam.z() > 0)
        if (markerPose.getOrigin().z() < 0)
        { 
            ROS_WARN("Detected marker with negetive z value");
            continue;
        }
        
        // 如果二维码识别有效, 且需要可视化, 则在图像上进行绘制
        CvPoint3D32f pw (markerPose.getOrigin().x(), markerPose.getOrigin().y(), markerPose.getOrigin().z());
        Pose pose;
        CvPoint2D32f pi;
        cam->ProjectPoint(pw, &pose, pi);
        if (is_visualization) {
          cv::circle(mat_image_ori, CvPoint(int(pi.x), int(pi.y)), 100, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }
        if (pi.x < 500 || pi.x > 1500) {
          ROS_INFO("NOT In ROI Region");
          continue;
        }

				// 获得到输出坐标系下的坐标位姿, 目前发布在map下
				tf::Transform tagPoseOutput = CamToOutput * markerPose;

				// 这里将识别到的ar_marker放入到数组里
				ar_track_alvar_msgs::AlvarMarker ar_pose_marker;
				tf::poseTFToMsg (tagPoseOutput, ar_pose_marker.pose.pose);
        // TODO: 如果识别到的码与上次位置距离非常远, 认为是识别到了非我们想去位置的码, 应当过滤; 同时, 此处的作用还能够将要识别的位置但在5m外处识别到时过滤, 防止那个时候不准
        if ((ar_pose_marker.pose.pose.position.x - last_ar_pose.pose.position.x) * (ar_pose_marker.pose.pose.position.x - last_ar_pose.pose.position.x) + 
            (ar_pose_marker.pose.pose.position.y - last_ar_pose.pose.position.y) * (ar_pose_marker.pose.pose.position.y - last_ar_pose.pose.position.y) < 25.0) 
            {
              ar_pose_marker.header.frame_id = output_frame;
              ar_pose_marker.header.stamp = image_msg->header.stamp;
              ar_pose_marker.id = id;
              arPoseMarkers_.markers.push_back (ar_pose_marker);
            }
			}

      if (is_visualization) {
        cv_ptr_->image = mat_image_ori;
        arImagePub_.publish(cv_ptr_->toImageMsg());
      }

      if (stop_for_fineture && (ros::Time::now().toSec() - last_coarse_detect_time > 4)) 
      {
        // 如果已经识别到粗ar位置并且停车, 正常情况下2s-4s内会发送一个新的finetune的结果, 但是如果在这段时间内, 恰好又识别不到ar码, 则在4s后, 发送coarse结果
        ROS_INFO("vehicle has stopped! but no fine tune result for a long time! use coarse result instead");
        arMarkerPubFine_.publish(coarse_ar_pose);
        last_ar_pose = coarse_ar_pose;
        stop_for_fineture = false;
        return;
      }

      geometry_msgs::PoseStamped ar_pose;

      if (arPoseMarkers_.markers.size() == 1)
      {
        // 如果只识别到一个码
        tf::Quaternion q_temp, q_work;

        /* 角度识别不是很准, 所以直接用初始的角度直接代替
        // 需要注意的是, ar码的平面朝向定义为z轴, 而不是想象的x轴, 需要绕x轴顺时针转90°, 再绕(旋转过后的)z轴转90°
        tf::quaternionMsgToTF(arPoseMarkers_.markers[0].pose.pose.orientation , q_temp);

        // setRPY绕xyz轴转, 但是参考轴不发生变化, 均是最初的参考轴
        q_work.setRPY(1.5708, 1.5708, 0);
        q_temp = q_work*q_temp;  
        q_temp.normalize(); // 归一化

        double roll, pitch, yaw, last_yaw;
        tf::Matrix3x3(q_temp).getRPY(roll, pitch, yaw);
        arPoseMarkers_.markers[0].pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);      // 投影到地面上
        */
        
        arPoseMarkers_.markers[0].pose.pose.orientation.x = coarse_pose.pose.orientation.x;
        arPoseMarkers_.markers[0].pose.pose.orientation.y = coarse_pose.pose.orientation.y;
        arPoseMarkers_.markers[0].pose.pose.orientation.z = coarse_pose.pose.orientation.z;
        arPoseMarkers_.markers[0].pose.pose.orientation.w = coarse_pose.pose.orientation.w;

        tf::quaternionMsgToTF(arPoseMarkers_.markers[0].pose.pose.orientation , q_temp);
        q_temp.normalize(); // 归一化

        double roll, pitch, yaw, last_yaw;
        tf::Matrix3x3(q_temp).getRPY(roll, pitch, yaw);   // yaw角在后面有用, 所以这步是需要的

        if (charge_status == NeedCharge) {
          arPoseMarkers_.markers[0].pose.pose.position.x += dist_before_charge * std::cos(yaw);  // 停靠在距离AR码0.8m之前, 认为到达终点
          arPoseMarkers_.markers[0].pose.pose.position.y += dist_before_charge * std::sin(yaw);
        }
        else if (charge_status == NeedTrash) {
          arPoseMarkers_.markers[0].pose.pose.position.x += dist_before_trash * std::cos(yaw);  // 停靠在距离AR码0.8m之前, 认为到达终点
          arPoseMarkers_.markers[0].pose.pose.position.y += dist_before_trash * std::sin(yaw);
        }

        ar_pose.header.frame_id = arPoseMarkers_.markers[0].header.frame_id;
        ar_pose.header.stamp = arPoseMarkers_.markers[0].header.stamp;
        ar_pose.pose = arPoseMarkers_.markers[0].pose.pose;

        visualizeMarker(ar_pose.pose, ar_pose.header);  // 在rviz中的可视化

        // 获取上一次发布pose的yaw角, 这里由于存储时已经经过转换, 所以没必要再经过一次转换
        tf::quaternionMsgToTF(last_ar_pose.pose.orientation , q_temp);
        tf::Matrix3x3(q_temp).getRPY(roll, pitch, last_yaw);  // roll 和 pitch不管

        if (stop_for_fineture && (ros::Time::now().toSec() - last_coarse_detect_time > 2)) 
        {
          // 如果已经识别到粗ar位置并且停车, 考虑到实际停车需要一些延迟, 目前设置为2s
          ROS_INFO("fine tune success!");
          arMarkerPubFine_.publish(ar_pose);
          last_ar_pose = ar_pose;
          stop_for_fineture = false;
          return;
        }

        // 与上一次定位的距离有0.2m以上的偏差,或方向角有5°的偏差, 则发布, 否则不发布使astar_navi不接受新消息, 由于角度永远给初始位姿的方向, 所以对角度的稳定性判断就不要了
        if ((ar_pose.pose.position.x - last_ar_pose.pose.position.x) * (ar_pose.pose.position.x - last_ar_pose.pose.position.x) + 
             (ar_pose.pose.position.y - last_ar_pose.pose.position.y) * (ar_pose.pose.position.y - last_ar_pose.pose.position.y) > 0.04)   // 这里写 5 / 180 * 3.1415926会变成0, 类型与LHS相同
        {
          if (ros::Time::now().toSec() - last_coarse_detect_time > 10) {
            // 在发布新终点的时候，需要再发布一个停车指令, 重新规划需要停车, 另外最好在检测到时, 也能停车, 这样能减少车辆抖动带来的干扰
            ROS_INFO("coarse pose detected! stop cmd publish!");
            end_point_status.vehicle_status.data = 2;
            vehicle_status_pub_.publish(end_point_status);
            coarse_ar_pose = ar_pose;
            stop_for_fineture = true;    // 记录粗糙识别位置, 此时需要停车, 来进行更准确的终点与起点定位, 待识别到精准位置后, 再发布
            last_coarse_detect_time = ros::Time::now().toSec();
          }
        }
      }
      
		}
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR ("Could not convert from '%s' to 'rgb8'.", image_msg->encoding.c_str ());
    }
	}
}

void configCallback(ar_track_alvar::ParamsConfig &config, uint32_t level)
{
  ROS_INFO("AR tracker reconfigured: %s %.2f %.2f %.2f %.2f", config.enabled ? "ENABLED" : "DISABLED",
           config.max_frequency, config.marker_size, config.max_new_marker_error, config.max_track_error);

  enableSwitched = enabled != config.enabled;

  enabled = config.enabled;
  max_frequency = config.max_frequency;
  marker_size = config.marker_size;
  max_new_marker_error = config.max_new_marker_error;
  max_track_error = config.max_track_error;
}

void enableCallback(const std_msgs::BoolConstPtr& msg)
{
    enableSwitched = enabled != msg->data;
    enabled = msg->data;
}

void dockCallback(const std_msgs::Int16::ConstPtr& msg)
{ 
  if (msg->data == NeedCharge) 
  {
    // 如果收到当前需要充电的信息, 发布最近的充电桩粗信息
    double max_distance = DBL_MAX;
    int charge_index = -1;
    for (int i = 0; i < CoarseChargingPositions.size(); i++)
    {
      double distance = std::abs(current_pose_global.pose.position.x - CoarseChargingPositions[i][0]) + std::abs(current_pose_global.pose.position.y - CoarseChargingPositions[i][1]);
      if (distance < max_distance) 
      {
        max_distance = distance;
        charge_index = i;
      }
    }
    autoware_msgs::DockInfo dock_pose;

    coarse_pose.pose.position.x = CoarseChargingPositions[charge_index][0];
    coarse_pose.pose.position.y = CoarseChargingPositions[charge_index][1];
    coarse_pose.pose.position.z = CoarseChargingPositions[charge_index][2];
    coarse_pose.pose.orientation.x = CoarseChargingPositions[charge_index][3];
    coarse_pose.pose.orientation.y = CoarseChargingPositions[charge_index][4];
    coarse_pose.pose.orientation.z = CoarseChargingPositions[charge_index][5];
    coarse_pose.pose.orientation.w = CoarseChargingPositions[charge_index][6];
    coarse_pose.header.frame_id = "world";   // 写死吧目前
    coarse_pose.header.stamp = ros::Time::now();
    last_ar_pose = coarse_pose;

    dock_pose.goal = coarse_pose;
    dock_pose.task = "charge";
    arMarkerPub_.publish(dock_pose);
    // arMarkerPub_.publish(coarse_pose);
  }

  else if (msg->data == NeedTrash)
  {
    // 如果收到当前需要倒垃圾的信息, 发布最近的垃圾桶粗信息
    double max_distance = DBL_MAX;
    int trash_index = -1;
    for (int i = 0; i < CoarseTrashbinPositions.size(); i++)
    {
      double distance = std::abs(current_pose_global.pose.position.x - CoarseTrashbinPositions[i][0]) + std::abs(current_pose_global.pose.position.y - CoarseTrashbinPositions[i][1]);
      if (distance < max_distance) 
      {
        max_distance = distance;
        trash_index = i;
      }
    }
    autoware_msgs::DockInfo dock_pose;

    coarse_pose.pose.position.x = CoarseTrashbinPositions[trash_index][0];
    coarse_pose.pose.position.y = CoarseTrashbinPositions[trash_index][1];
    coarse_pose.pose.position.z = CoarseTrashbinPositions[trash_index][2];
    coarse_pose.pose.orientation.x = CoarseTrashbinPositions[trash_index][3];
    coarse_pose.pose.orientation.y = CoarseTrashbinPositions[trash_index][4];
    coarse_pose.pose.orientation.z = CoarseTrashbinPositions[trash_index][5];
    coarse_pose.pose.orientation.w = CoarseTrashbinPositions[trash_index][6];
    coarse_pose.header.frame_id = "world";   // 写死吧目前
    coarse_pose.header.stamp = ros::Time::now();
    last_ar_pose = coarse_pose;

    dock_pose.goal = coarse_pose;
    dock_pose.task = "garbage";
    arMarkerPub_.publish(dock_pose);
    // arMarkerPub_.publish(coarse_pose);
  }

  charge_status = static_cast<DockState>(msg->data);   // 在发布粗略终点后再切状态
}

void currentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  current_pose_global = msg;
}

int main(int argc, char *argv[])
{
	ros::init (argc, argv, "marker_detect");
	ros::NodeHandle n, pn("~");

  if(argc > 1) {
    ROS_WARN("Command line arguments are deprecated. Consider using ROS parameters and remappings.");

    if(argc < 7){
      std::cout << std::endl;
      cout << "Not enough arguments provided." << endl;
      cout << "Usage: ./individualMarkersNoKinect <marker size in cm> <max new marker error> <max track error> "
           << "<cam image topic> <cam info topic> <output frame> [ <max frequency> <marker_resolution> <marker_margin>]";
      std::cout << std::endl;
      return 0;
    }

    // Get params from command line
    marker_size = atof(argv[1]);
    max_new_marker_error = atof(argv[2]);
    max_track_error = atof(argv[3]);
    cam_image_topic = argv[4];
    cam_info_topic = argv[5];
    output_frame = argv[6];

    if (argc > 7) {
      max_frequency = atof(argv[7]);
      pn.setParam("max_frequency", max_frequency);
    }
    if (argc > 8)
      marker_resolution = atoi(argv[8]);
    if (argc > 9)
      marker_margin = atoi(argv[9]);

  } else {
    // Get params from ros param server.
    pn.param("marker_size", marker_size, 10.0);
    pn.param("max_new_marker_error", max_new_marker_error, 0.08);
    pn.param("max_track_error", max_track_error, 0.2);
    pn.param("max_frequency", max_frequency, 8.0);
    pn.setParam("max_frequency", max_frequency);  // in case it was not set.
    pn.param("marker_resolution", marker_resolution, 5);
    pn.param("marker_margin", marker_margin, 2);
    if (!pn.getParam("output_frame", output_frame)) {
      ROS_ERROR("Param 'output_frame' has to be set.");
      exit(EXIT_FAILURE);
    }

    // Camera input topics. Use remapping to map to your camera topics.
    cam_image_topic = "camera_image";
    cam_info_topic = "camera_info";
  }

  // Set dynamically configurable parameters so they don't get replaced by default values
  pn.setParam("marker_size", marker_size);
  pn.setParam("max_new_marker_error", max_new_marker_error);
  pn.setParam("max_track_error", max_track_error);
  pn.param("dist_before_charge", dist_before_charge, 1.2);
  pn.param("dist_before_trash", dist_before_trash, 2.0);
  pn.param("charge_id", charge_id, 9);
  pn.param("trash_id", trash_id, 8);
  pn.param("debug_mode", debug_mode, false);

	marker_detector.SetMarkerSize(marker_size, marker_resolution, marker_margin);

	cam = new Camera(n, cam_info_topic);
	tf_listener = new tf::TransformListener(n);
	tf_broadcaster = new tf::TransformBroadcaster();
	// arMarkerPub_ = n.advertise < ar_track_alvar_msgs::AlvarMarkers > ("ar_pose_marker", 0);
  arImagePub_ = n.advertise < sensor_msgs::Image > ("image_with_ar", 0);
  // arMarkerPub_ = n.advertise < geometry_msgs::PoseStamped > ("move_base_simple/goal", 0);
  arMarkerPub_ = n.advertise < autoware_msgs::DockInfo > ("move_base_simple/goal", 0);
  arMarkerPubFine_ = n.advertise < geometry_msgs::PoseStamped > ("move_base_simple/goal_fine", 0);
	rvizMarkerPub_ = n.advertise < visualization_msgs::Marker > ("visualization_marker", 0);
  vehicle_status_pub_ = n.advertise<autoware_msgs::DrivingStatus>("driving_status", 0);

  // Prepare dynamic reconfiguration
  dynamic_reconfigure::Server < ar_track_alvar::ParamsConfig > server;
  dynamic_reconfigure::Server<ar_track_alvar::ParamsConfig>::CallbackType f;

  f = boost::bind(&configCallback, _1, _2);
  server.setCallback(f);

	//Give tf a chance to catch up before the camera callback starts asking for transforms
  // It will also reconfigure parameters for the first time, setting the default values
	ros::Duration(1.0).sleep();
	ros::spinOnce();

	image_transport::ImageTransport it_(n);

  // Run at the configured rate, discarding pointcloud msgs if necessary
  ros::Rate rate(max_frequency);

  /// Subscriber for enable-topic so that a user can turn off the detection if it is not used without
  /// having to use the reconfigure where he has to know all parameters
  ros::Subscriber enable_sub_ = pn.subscribe("enable_detection", 1, &enableCallback);
  ros::Subscriber dock_sub = pn.subscribe("dock_status", 1, &dockCallback);
  ros::Subscriber current_pose_sub = pn.subscribe("current_pose", 1, &currentPoseCallback);

  enableSwitched = true;

  // 初始化last_ar_pose
  last_ar_pose.pose.position.x = 0;
  last_ar_pose.pose.position.y = 0;
  last_ar_pose.pose.position.z = 0;
  last_ar_pose.pose.orientation.x = 0;
  last_ar_pose.pose.orientation.y = 0;
  last_ar_pose.pose.orientation.z = 0;
  last_ar_pose.pose.orientation.w = 1;
  
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();

    if (std::abs((rate.expectedCycleTime() - ros::Duration(1.0 / max_frequency)).toSec()) > 0.001)
    {
      // Change rate dynamically; if must be above 0, as 0 will provoke a segfault on next spinOnce
      ROS_DEBUG("Changing frequency from %.2f to %.2f", 1.0 / rate.expectedCycleTime().toSec(), max_frequency);
      rate = ros::Rate(max_frequency);
    }

    if (enableSwitched)
    {
      // Enable/disable switch: subscribe/unsubscribe to make use of pointcloud processing nodelet
      // lazy publishing policy; in CPU-scarce computer as TurtleBot's laptop this is a huge saving
        if (enabled)
            cam_sub_ = it_.subscribe(cam_image_topic, 1, &getCapCallback);
        else
            cam_sub_.shutdown();
        enableSwitched = false;
    }

    if (debug_mode) 
    { 
      // 只进一次, 先后发送倒垃圾、充电的位置
      debug_mode = false;

      autoware_msgs::DockInfo dock_pose;
      // TODO: 发送倒垃圾, 在这里填写rviz中勾取的粗略垃圾桶位置
      coarse_pose.pose.position.x = -3.47; 
      coarse_pose.pose.position.y = -5.18;
      coarse_pose.pose.position.z = 0;
      coarse_pose.pose.orientation.x = 0;
      coarse_pose.pose.orientation.y = 0;
      coarse_pose.pose.orientation.z = 0.826;
      coarse_pose.pose.orientation.w = 0.563;
      coarse_pose.header.frame_id = "world";   // 写死吧目前
      coarse_pose.header.stamp = ros::Time::now();

      dock_pose.goal = coarse_pose;
      dock_pose.task = "garbage";
      arMarkerPub_.publish(dock_pose);

      // 延时2s后, 发布充电桩位置
      ros::Duration(2.0).sleep();
      coarse_pose.pose.position.x = 19.0; 
      coarse_pose.pose.position.y = 1.73;
      coarse_pose.pose.position.z = 0;
      coarse_pose.pose.orientation.x = 0;
      coarse_pose.pose.orientation.y = 0;
      coarse_pose.pose.orientation.z = 0.845;
      coarse_pose.pose.orientation.w = 0.534;
      coarse_pose.header.frame_id = "world";   // 写死吧目前
      coarse_pose.header.stamp = ros::Time::now();

      dock_pose.goal = coarse_pose;
      dock_pose.task = "charge";
      arMarkerPub_.publish(dock_pose);

      // charge_status = NeedCharge; // 人为修改状态, 开启检测, 这样就不需要手动给充电命令
    }

  }

    return 0;
}
