#include "ros/ros.h"
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <jsoncpp/json/json.h>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>  
#include <pix_driver_msgs/VcuReport.h>
#include <pix_driver_msgs/GearReport.h>
#include <pix_driver_msgs/BmsReport.h>
#include <pix_driver_msgs/SteerReport.h>
#include <pix_driver_msgs/ThrottleReport.h>
#include <autoware_msgs/DrivingStatus.h>

using namespace std;

class AppClient
{
public:
    // 构造函数
	AppClient() : nh_(nh) {
        init();
    }
    ~AppClient() = default;

	void recvLoop();

    // SendDebugInfo()负责把pix底盘数据发送给app可视化
	void SendDebugInfo();

    int client_flag;

    ros::NodeHandle nh;

private:

	// control()负责远程控制信号解读
	void control(int &sig);

    void init();

    // pix底盘数据 订阅的回调函数
    void vcu_callback(const pix_driver_msgs::VcuReport::ConstPtr &msg);
    void gear_callback(const pix_driver_msgs::GearReport::ConstPtr &msg);
    void bms_callback(const pix_driver_msgs::BmsReport::ConstPtr &msg);
    void steer_callback(const pix_driver_msgs::SteerReport::ConstPtr &msg);
    void throttle_callback(const pix_driver_msgs::ThrottleReport::ConstPtr &msg);
    // gps数据的回调函数
    void GpsCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    // 车辆模式状态的回调函数
    void modeCallback(const std_msgs::Int32::ConstPtr &msg);
    // 车辆是否自驾状态的回调函数
    void autoCallback(const std_msgs::Int32::ConstPtr &msg);
	// 终点队列是否为空的回调函数
    void goalCallback(const std_msgs::Int32::ConstPtr &msg);

    //pix订阅句柄
    ros::Subscriber sub_vcu;
    ros::Subscriber sub_gear;
    ros::Subscriber sub_bms;
    ros::Subscriber sub_steer;
    ros::Subscriber sub_throttle;
    // gps订阅句柄
    ros::Subscriber sub_gps;
    // 车辆模式状态订阅句柄
    ros::Subscriber sub_mode;
    // 车辆自驾状态的订阅
    ros::Subscriber sub_auto;
	// 终点队列是否为空的订阅
	ros::Subscriber sub_goal;

    ros::Publisher pub_cleaning_cmd;

    // 控制信息发布句柄
	// ros::Publisher chatter_pub;
	// ros::Publisher control_pub;
	// ros::Publisher vehicle_mode_pub;
	// ros::Publisher throttle_pub;
	// ros::Publisher brake_pub;
	// ros::Publisher steer_pub;
	// ros::Publisher gear_pub;

	ros::NodeHandle nh_;

	Json::Reader reader;
	Json::FastWriter first_write;
	Json::FastWriter write;

	Json::Value json_temp;
	Json::Value json_temp_2;
	Json::Value json_temp_3;
	Json::Value val;

	char buf[2048];

	string SendBuf;  // 用于车辆
    // string SendBuf2;  // 用于GPS数据的socket发送
	string SendBuf3;  // // 用于车辆底盘数据的socket发送

    //接收pix底盘数据 变量
    string speed;
    string acc;
    int8_t vehicle_mode;
    string gear;
    string bms;
    string steer;
    string throttle;
    // 接收gnss变量
	string latitude;
	string longitude;
    // 接收车辆状态信息
    string mode_order;
    // 接受车辆是否处于自驾状态（暂时是 规划部分传来的）
    string auto_mode;
	// 接收终点队列是否为空
	string goal_status = "1";

	double last_pub_autodrive_time = 0.0;
	double last_pub_goalstatus_time = 0.0;

    autoware_msgs::DrivingStatus cleaning_cmd;

	// 控制消息
	// pix_driver_msgs::BrakeCommand brake_msg;
	// pix_driver_msgs::GearCommand gear_msg;
	// pix_driver_msgs::SteeringCommand steer_msg;
	// pix_driver_msgs::ThrottleCommand throttle_msg;
	// pix_driver_msgs::VehicleModeCommand vehicle_mode_msg;
};