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

// 第二代底盘驱动的 消息类型 和 第一代不一样，这是第二代的
#include "pix_driver_msgs/steering_report_502.h"
#include "pix_driver_msgs/gear_report_503.h"
#include "pix_driver_msgs/vcu_report_505.h"
#include "pix_driver_msgs/bms_report_512.h"
#include "pix_driver_msgs/throttle_report_500.h"



using namespace std;

string doubleTostring(double val)
{
	char *chCode;
	chCode = new char[30];
	sprintf(chCode, "%.2lf", val);
	string str(chCode);
	delete chCode;
	return str;
}

class client
{
public:
	client(ros::NodeHandle nh) : nh_(nh)
	{
		json_temp["chatType"] = 1;
		json_temp["content"] = "&";
		json_temp["errorCode"] = 0;
		json_temp["friendID"] = 1;
		json_temp["groupID"] = 1;
		json_temp["isFirst"] = true;
		json_temp["isGroup"] = false;
		json_temp["messageID"] = 1;
		json_temp["userID"] = 2;
		json_temp["userName"] = "client";


		json_temp_3["chatType"] = 42;
		json_temp_3["content"] = " , , , , , , , , , , , "; //初始点的位移
		json_temp_3["errorCode"] = 0;
		json_temp_3["friendID"] = 1;
		json_temp_3["groupID"] = 1;
		json_temp_3["isFirst"] = false;
		json_temp_3["isGroup"] = false;
		json_temp_3["messageID"] = 1;
		json_temp_3["userID"] = 2;
		json_temp_3["userName"] = "client";


		// socket通信
		std::cout << "This is client" << std::endl;
		// socket
		client_flag = socket(AF_INET, SOCK_STREAM, 0);
		if (client_flag == -1)
		{
			std::cout << "Error: socket" << std::endl;
			exit(-1);
		}
		// connect
		struct sockaddr_in serverAddr;
		serverAddr.sin_family = AF_INET;
		serverAddr.sin_port = htons(7777);  // 7777
		serverAddr.sin_addr.s_addr = inet_addr("8.130.40.126"); 
		if (connect(client_flag, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
		{
			std::cout << "Error: connect" << std::endl;
			exit(-1);
		}
		std::cout << "...connect" << std::endl;

		// SendBuf负责chattype的传递
		SendBuf = first_write.write(json_temp);
		// SendBuf3负责的是pix底盘的可视化数据
		SendBuf3 = write.write(json_temp_3);

		send(client_flag, SendBuf.c_str(), SendBuf.size(), 0);
        // send(client_flag, SendBuf2.c_str(), SendBuf2.size(), 0);
		send(client_flag, SendBuf3.c_str(), SendBuf3.size(), 0);

		memset(buf, 0, sizeof(buf));



		// // pix控制话题 发布句柄
		// vehicle_mode_pub = nh_.advertise<pix_driver_msgs::VehicleModeCommand>("/vehicle_mode_command", 100);
		// throttle_pub = nh_.advertise<pix_driver_msgs::ThrottleCommand>("/throttle_command", 100);
		// brake_pub = nh_.advertise<pix_driver_msgs::BrakeCommand>("/brake_command", 100);
		// steer_pub = nh_.advertise<pix_driver_msgs::SteeringCommand>("/steering_command", 100);
		// gear_pub = nh_.advertise<pix_driver_msgs::GearCommand>("/gear_command", 100);

        // pix底盘数据订阅
        sub_vcu = nh.subscribe<pix_driver_msgs::vcu_report_505>("/pix/vcu_report", 1, &client::vcu_callback, this);
        sub_gear = nh.subscribe<pix_driver_msgs::gear_report_503>("/pix/gear_report", 1, &client::gear_callback, this);
        sub_bms = nh.subscribe<pix_driver_msgs::bms_report_512>("/pix/bms_report", 1, &client::bms_callback, this);
        sub_steer = nh.subscribe<pix_driver_msgs::steering_report_502>("/pix/steering_report", 1, &client::steer_callback, this);
        sub_throttle = nh.subscribe<pix_driver_msgs::throttle_report_500>("/pix/throttle_report", 1, &client::throttle_callback, this);
        // gps经纬度数据订阅
        sub_gps = nh.subscribe<std_msgs::Float64MultiArray>("/lat_lon", 1, &client::GpsCallback, this);
        // 车辆模式状态的订阅（中心行驶、贴边、巡检）
        sub_mode = nh.subscribe<std_msgs::Int32>("/mode_order", 1, &client::modeCallback, this);
	}

	void recvLoop()
	{
		ros::Rate r(100);
		while (ros::ok())
		{
			int len = recv(client_flag, buf, sizeof(buf), 0);
			if (len > 0)
			{
				if (reader.parse(buf, val))
				{
					string str = val["userName"].asString();
					string str2 = val["content"].asString();
					int sig = val["chatType"].asInt();
					std::cout << "FROM:" << str << std::endl;
					std::cout << "CONTENT:" << str2 << std::endl;
					std::cout << "MSG:" << sig << std::endl;

					//0-STOP 10-目标点P 11-目标点A 12-目标点B 20-跟踪行人模式关闭
					//21-跟踪行人模式打开 30-返回循迹模式 31-进入遥控模式 4-油门 5-刹车 6-R 7-N 8-D 90-左 91-右
					control(sig);
				}
			}
			r.sleep();
		}
	}

	// control()负责远程控制信号解读
	void control(int &sig);
	// SendDebugInfo()负责把pix底盘数据发送给app可视化
	void SendDebugInfo();

    // pix底盘数据 订阅的回调函数
    void vcu_callback(const pix_driver_msgs::vcu_report_505ConstPtr &msg);
    void gear_callback(const pix_driver_msgs::gear_report_503ConstPtr &msg);
    void bms_callback(const pix_driver_msgs::bms_report_512ConstPtr &msg);
    void steer_callback(const pix_driver_msgs::steering_report_502ConstPtr &msg);
    void throttle_callback(const pix_driver_msgs::throttle_report_500ConstPtr &msg);
    // gps数据的回调函数
    void GpsCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    // 车辆模式状态的回调函数
    void modeCallback(const std_msgs::Int32::ConstPtr &msg);



	ros::NodeHandle nh_;

	Json::Reader reader;
	Json::FastWriter first_write;
	Json::FastWriter write;

	Json::Value json_temp;
	Json::Value json_temp_2;
	Json::Value json_temp_3;
	Json::Value val;

	int client_flag;

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

	// // 控制信息发布句柄
	// ros::Publisher chatter_pub;
	// ros::Publisher control_pub;
	// ros::Publisher vehicle_mode_pub;
	// ros::Publisher throttle_pub;
	// ros::Publisher brake_pub;
	// ros::Publisher steer_pub;
	// ros::Publisher gear_pub;

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

	// // 控制消息
	// pix_driver_msgs::BrakeCommand brake_msg;
	// pix_driver_msgs::GearCommand gear_msg;
	// pix_driver_msgs::SteeringCommand steer_msg;
	// pix_driver_msgs::ThrottleCommand throttle_msg;
	// pix_driver_msgs::VehicleModeCommand vehicle_mode_msg;

};


// pix底盘数据可视化
void client::vcu_callback(const pix_driver_msgs::vcu_report_505ConstPtr &msg)
{
    speed = doubleTostring(msg->Vehicle_Speed);
    acc = doubleTostring(msg->Vehicle_Acc);
    vehicle_mode = msg->Vehicle_ModeState;  // 这是车辆的模式状态：0x3: Standby Mode；0x2: Emergency Mode；0x1: Auto Mode；0x0: Manual Remote Mode
}

void client::gear_callback(const pix_driver_msgs::gear_report_503ConstPtr &msg)
{
    gear = to_string(msg->Gear_Actual);
}

void client::bms_callback(const pix_driver_msgs::bms_report_512ConstPtr &msg)
{
    bms = to_string(msg->Battery_Soc);
}

void client::steer_callback(const pix_driver_msgs::steering_report_502ConstPtr &msg)
{
    steer = to_string(msg->Steer_AngleActual);
}

void client::throttle_callback(const pix_driver_msgs::throttle_report_500ConstPtr &msg)
{
    throttle = doubleTostring(msg->Dirve_ThrottlePedalActual);
}

// gps数据处理（发送）
void client::GpsCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    // std::cout << "receive gps";
	double lat = msg->data[0], lon = msg->data[1];
    // ROS_INFO_STREAM(lat << " " << lon);  // ROS_INFO_STREAM和cout都会强制将输出限制成6位，实际上所有的位数都还在
    // std::cout << lat << endl;
    // std::cout << lon << endl;
	lat = lat * 10000000;  // 为了 在转成string的时候不会损失小数位数，所以让整数位尽可能多
	lon = lon * 10000000;
	latitude = doubleTostring(lat);
	longitude = doubleTostring(lon);
	// ROS_INFO_STREAM(latitude);  // 打印测试
	// string content = latitude + "," + longitude;
	// // printf("pub info %s \n", content.c_str());
	// json_temp_2["content"] = content;

	// SendBuf2 = write.write(json_temp_2);
	// send(client_flag, SendBuf2.c_str(), SendBuf2.size(), 0);
	// // json_temp_2["content"] = "21,23,26.730899,119.570362";
}

// 车辆模式状态 的显示
void client::modeCallback(const std_msgs::Int32::ConstPtr &msg)
{
    int32_t mode = msg->data;
    mode_order = doubleTostring(double(mode));
}

// 底盘数据的发送
void client::SendDebugInfo()
{
	static int counter = 0;
	// 发给app的可视化的字段
    string nn = "none";
    string content = bms + "," +nn + "," +
					throttle + "," + nn + "," +
					gear + "," + nn + "," +
					speed + "," + nn + "," +
					steer + "," + nn + "," +
					acc + "," + nn + "," +
					latitude + "," + nn + "," +
					longitude + "," + nn + "," +
                    mode_order + "," + nn;
	json_temp_3["content"] = content;
	// printf("pub info %s \n", content.c_str());
	SendBuf3 = write.write(json_temp_3);
	send(client_flag, SendBuf3.c_str(), SendBuf3.size(), 0);

	counter++;
	if (counter == 8)
		counter = 0;
}


void client::control(int &sig)
{
	if (sig == 0)  // stop指令，车停下
	{
		// brake_msg.brake_en_ctrl = 1;  // 刹车使能
		// brake_msg.brake_pedal_target = 50;  // 刹车量50% 
		// vehicle_mode_msg.vin_req = 1;
		// vehicle_mode_msg.drive_mode_ctrl = 1;  // 设置用速度量控制车速
		// throttle_msg.throttle_en_ctrl = 1;
		// throttle_msg.vel_target = 0;  // 车速为零
		// throttle_msg.throttle_pedal_target = 0;  // 保险起见，油门量为0
	}

	if (sig == 10)  // 中心模式 -- 0
	{
		FILE *fp0 = popen("rostopic pub -1 /mode_order std_msgs/Int32 -- '0' ", "r");
		pclose(fp0);
	}
	if (sig == 11)  // 贴边模式
	{
		FILE *fp1 = popen("rostopic pub -1 /mode_order std_msgs/Int32 -- '1' ", "r");
		pclose(fp1);
	}
	if (sig == 12)  // 巡检模式
	{
		FILE *fp2 = popen("rostopic pub -1 /mode_order std_msgs/Int32 -- '2' ", "r");
		pclose(fp2);
	}
	if (sig == 13)  // 第四种状态的预留...
	{
		// 
	}
	if (sig == 21)  // 正常模式（可以切换贴边、巡检）启动
	{
		FILE *fp;
		// char buffer[80];
		fp=popen("~/autoware.gf/autoware-1.14/normal.sh", "r");    //以读方式，fork产生一个子进程，执行shell命令
		// fgets(buffer,sizeof(buffer), fp);       //读取shell脚本中输出(stdout)的值
		// printf("%s", buffer);
		pclose(fp);
	}
	if (sig == 20)  // 正常模式关闭（暂未实现）
	{
		//  
	}
	if (sig == 121)  // 全覆盖模式启动
	{
		FILE *fp;
		fp=popen("~/autoware.gf/autoware-1.14/cover.sh", "r");    //以读方式，fork产生一个子进程，执行shell命令
		pclose(fp);
	}
	if (sig == 120)  // 全覆盖模式关闭（暂未实现）
	{
		//
	}
	if (sig == 31)  // 打开遥控模式
	{
		// // 先设置控制模式：
		// vehicle_mode_msg.vin_req = 1;  // 使能
		// vehicle_mode_msg.drive_mode_ctrl = 0;  // 用throttle_pedal_drive（油门量）控制速度
		// vehicle_mode_msg.steer_mode_ctrl = 0;  // 用standard_steer 控制角度

		// // 打开手动档位：
		// gear_msg.gear_en_ctrl = 1;  // 使能
		// gear_msg.gear_target = 3;  // neutral状态
	}
	if (sig == 30)  // 关闭遥控模式（假关闭！只是让速度等于0）
	{
		// vehicle_mode_msg.vin_req = 1;
		// vehicle_mode_msg.drive_mode_ctrl = 1;  // 设置用速度量控制车速
		// throttle_msg.throttle_en_ctrl = 1;
		// throttle_msg.vel_target = 0;  // 车速为零
		// throttle_msg.throttle_pedal_target = 0;  // 保险起见，油门量为0
	}
	// // 控制信息发布
	// vehicle_mode_pub.publish(vehicle_mode_msg);
	// gear_pub.publish(gear_msg);
	// throttle_pub.publish(throttle_msg);
	// brake_pub.publish(brake_msg);

	if (vehicle_mode  == 0)  // 如果进入遥控模式，可以用油门、刹车、转向
	{
		if (sig == 4)  // 加速
		{
			// vehicle_mode_msg.vin_req = 1;
			// vehicle_mode_msg.drive_mode_ctrl = 0;  // 设置用油门量控制车速
			// brake_msg.brake_en_ctrl = 1;  // 刹车使能
			// brake_msg.brake_pedal_target = 0;  // 刹车量0% 
			// throttle_msg.throttle_en_ctrl = 1;  // 使能
			// throttle_msg.throttle_pedal_target += 10;  // 油门量
			// // throttle_msg.throttle_pedal_target = throttle_msg.throttle_pedal_target >= 0 ? throttle_msg.throttle_pedal_target : 0;  
			// throttle_msg.throttle_pedal_target = throttle_msg.throttle_pedal_target < 100 ? throttle_msg.throttle_pedal_target : 100;  // 油门量限幅0-100%
			
			sig = 999;
		}
		if (sig == 5)  // 刹车
		{
			// throttle_msg.throttle_en_ctrl = 1;  // 使能
			// throttle_msg.throttle_pedal_target -= 10;  // 油门量
			// throttle_msg.throttle_pedal_target = throttle_msg.throttle_pedal_target >= 0 ? throttle_msg.throttle_pedal_target : 0;  // 刹车到最小，油门量归零
			// brake_msg.brake_en_ctrl = 1;  // 刹车使能
			// brake_msg.brake_pedal_target = 50;  // 刹车量50% 

			sig = 999;
		}
		if (sig == 90)  // 左传
		{
			// steer_msg.steer_en_ctrl = 1;  // 使能
			// steer_msg.steer_angle_target -= 50;  // 舵机量
			// steer_msg.steer_angle_target = steer_msg.steer_angle_target <= -360 ? -360 : steer_msg.steer_angle_target;
			
			sig = 999;
		}
		if (sig == 91)  // 右转
		{
			// steer_msg.steer_en_ctrl = 1;  // 使能
			// steer_msg.steer_angle_target += 50;  // 舵机量
			// steer_msg.steer_angle_target = steer_msg.steer_angle_target >= 360 ? 360 : steer_msg.steer_angle_target;

			sig = 999;
		}
		if (sig == 6)  // R档 
		{
			// gear_msg.gear_en_ctrl = 1;  // 使能
			// gear_msg.gear_target = 2;  // reverse

			sig = 999;
		}
		if (sig == 7)  // N档
		{
			// gear_msg.gear_en_ctrl = 1;  // 使能
			// gear_msg.gear_target = 1;  // park

			sig = 999;
		}
		if (sig == 8)  // D档
		{
			// gear_msg.gear_en_ctrl = 1;  // 使能
			// gear_msg.gear_target = 3;  // neutral

			sig = 999;
		}
	}
	else
	{
		// gear_msg.gear_target = 3;  // neutral
		// steer_msg.steer_angle_target = 0;  // 舵机量
		// throttle_msg.throttle_pedal_target = 0;  // 油门量

	}
	// // 控制信息发布
	// vehicle_mode_pub.publish(vehicle_mode_msg);
	// gear_pub.publish(gear_msg);
	// throttle_pub.publish(throttle_msg);
	// brake_pub.publish(brake_msg);
	// steer_pub.publish(steer_msg);
 
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "sub_pub");
	ros::NodeHandle nh;

	client myclient(nh);
	ros::Rate loop_rate(100);

	std::thread recv_thread(&client::recvLoop, &myclient);

	while (ros::ok())
	{
		ros::spinOnce();
		myclient.SendDebugInfo();
		loop_rate.sleep();
	}
	cout << "...closing" << endl;
	shutdown(myclient.client_flag, SHUT_RDWR);
	close(myclient.client_flag);
	cout << "...closed" << endl;

	recv_thread.join();
	return 0;
}
