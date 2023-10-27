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
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 */
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

class CloudTransformerNode
{
private:

	ros::NodeHandle     node_handle_;
	ros::Subscriber     points_node_sub_;
	ros::Publisher      transformed_points_pub_;

	std::string         input_point_topic_;
	std::string         target_frame_;
	std::string         output_point_topic_;

	tf::TransformListener *tf_listener_ptr_;

	bool                transform_ok_;

	void publish_cloud(const ros::Publisher& in_publisher,
	                   const sensor_msgs::PointCloud2& in_cloud_msg)
	{
		in_publisher.publish(in_cloud_msg);
	}

	void transformXYZIRCloud(const pcl::PointCloud<pcl::PointXYZRGB>& in_cloud,
	                         pcl::PointCloud<pcl::PointXYZRGB>& out_cloud,
	                         const tf::StampedTransform& in_tf_stamped_transform)
	{
		Eigen::Matrix4f transform;
		pcl_ros::transformAsMatrix(in_tf_stamped_transform, transform);
		pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(in_cloud, *in_cloud_XYZ);
		pcl::transformPointCloud (*in_cloud_XYZ, *transformed_cloud_XYZ, transform); 
		pcl::copyPointCloud(*transformed_cloud_XYZ, out_cloud);
		for (int i = 0; i < (int)out_cloud.points.size(); i++)
  		{
			out_cloud.points[i].r = in_cloud.points[i].r;
			out_cloud.points[i].g = in_cloud.points[i].g;
			out_cloud.points[i].b = in_cloud.points[i].b;
		}
	}

	void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
	{
		sensor_msgs::PointCloud2 transformed_cloud;

		bool do_transform = false;
		tf::StampedTransform transform;
		if (target_frame_ != in_sensor_cloud->header.frame_id)
		{
			try {
				tf_listener_ptr_->lookupTransform(target_frame_, in_sensor_cloud->header.frame_id, ros::Time(0),
				                                  transform);
				do_transform = true;
			}
			catch (tf::TransformException ex) {
				ROS_ERROR("cloud_transformer: %s NOT Transforming.", ex.what());
				do_transform = false;
				transform_ok_ = false;
			}
		}
		if (do_transform)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::fromROSMsg(*in_sensor_cloud, *in_cloud_ptr);
			transformXYZIRCloud(*in_cloud_ptr, *transformed_cloud_ptr, transform);
			pcl::toROSMsg(*transformed_cloud_ptr, transformed_cloud);
			transformed_cloud.header.frame_id = target_frame_;
			if (!transform_ok_)
				{ROS_INFO("cloud_transformer: Correctly Transformed"); transform_ok_=true;}
		}
		else
			{ transformed_cloud = *in_sensor_cloud;}

		publish_cloud(transformed_points_pub_, transformed_cloud);
	}

public:
	CloudTransformerNode(tf::TransformListener* in_tf_listener_ptr):node_handle_("~"), transform_ok_(false)
	{
		tf_listener_ptr_ = in_tf_listener_ptr;
	}
	void Run()
	{
		ROS_INFO("Initializing Cloud Transformer, please wait...");
		node_handle_.param<std::string>("input_point_topic", input_point_topic_, "/points_raw");
		ROS_INFO("Input point_topic: %s", input_point_topic_.c_str());

		node_handle_.param<std::string>("target_frame", target_frame_, "velodyne");
		ROS_INFO("Target Frame in TF (target_frame) : %s", target_frame_.c_str());

		node_handle_.param<std::string>("output_point_topic", output_point_topic_, "/points_transformed");
		ROS_INFO("output_point_topic: %s", output_point_topic_.c_str());

		ROS_INFO("Subscribing to... %s", input_point_topic_.c_str());
		points_node_sub_ = node_handle_.subscribe(input_point_topic_, 1, &CloudTransformerNode::CloudCallback, this);

		transformed_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(output_point_topic_, 2);

		ROS_INFO("Ready");

		ros::spin();

	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cloud_transformer");
	tf::TransformListener tf_listener;
	CloudTransformerNode app(&tf_listener);

	app.Run();

	return 0;

}
