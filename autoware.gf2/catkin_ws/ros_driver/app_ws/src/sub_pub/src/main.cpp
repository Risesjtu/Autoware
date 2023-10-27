#include "main.h"


string doubleTostring(double val)
{
	char *chCode;
	chCode = new char[30];
	sprintf(chCode, "%.2lf", val);
	string str(chCode);
	delete chCode;
	return str;
}

void AppClient::init() 
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
	serverAddr.sin_port = htons(6868);  // 7777
	serverAddr.sin_addr.s_addr = inet_addr("8.130.40.126"); 
	if (connect(client_flag, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
	{
		std::cout << "Error: connect" << std::endl;
		exit(-1);
	}
	std::cout << "The car correctly connect with app !" << std::endl;

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
	sub_vcu = nh.subscribe<pix_driver_msgs::VcuReport>("/pix/vcu_report", 1, &AppClient::vcu_callback, this);
	sub_gear = nh.subscribe<pix_driver_msgs::GearReport>("/pix/gear_report", 1, &AppClient::gear_callback, this);
	sub_bms = nh.subscribe<pix_driver_msgs::BmsReport>("/pix/bms_report", 1, &AppClient::bms_callback, this);
	sub_steer = nh.subscribe<pix_driver_msgs::SteerReport>("/pix/steer_report", 1, &AppClient::steer_callback, this);
	sub_throttle = nh.subscribe<pix_driver_msgs::ThrottleReport>("/pix/throttle_report", 1, &AppClient::throttle_callback, this);
	// gps经纬度数据订阅
	sub_gps = nh.subscribe<std_msgs::Float64MultiArray>("/lat_lon", 1, &AppClient::GpsCallback, this);
	// 车辆模式状态的订阅（中心行驶、贴边、巡检）
	sub_mode = nh.subscribe<std_msgs::Int32>("/mode_order", 1, &AppClient::modeCallback, this);
	// 车辆是否处于自动驾驶的订阅
	sub_auto = nh.subscribe<std_msgs::Int32>("/is_auto_driving", 1, &AppClient::autoCallback, this);
	// 终点队列是否为空的订阅
	sub_goal = nh.subscribe<std_msgs::Int32>("/is_goal_empty", 1, &AppClient::goalCallback, this);

	pub_cleaning_cmd = nh_.advertise<autoware_msgs::DrivingStatus>("driving_status", 1);
}

// 子线程调用的函数
void AppClient::recvLoop()
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

				// 对app发来的消息进行处理
				control(sig);
			}
		}
		r.sleep();
	}
}

// pix底盘数据可视化
void AppClient::vcu_callback(const pix_driver_msgs::VcuReport::ConstPtr &msg)
{
    speed = doubleTostring(msg->speed);
    acc = doubleTostring(msg->acc);
    vehicle_mode = msg->vehicle_mode_state;  // 这是车辆的模式状态：0x3: Standby Mode；0x2: Emergency Mode；0x1: Auto Mode；0x0: Manual Remote Mode
}

void AppClient::gear_callback(const pix_driver_msgs::GearReport::ConstPtr &msg)
{
    gear = to_string(msg->gear_actual);
}

void AppClient::bms_callback(const pix_driver_msgs::BmsReport::ConstPtr &msg)
{
    bms = to_string(msg->battery_soc);
}

void AppClient::steer_callback(const pix_driver_msgs::SteerReport::ConstPtr &msg)
{
    steer = to_string(msg->steer_angle_actual);
}

void AppClient::throttle_callback(const pix_driver_msgs::ThrottleReport::ConstPtr &msg)
{
    throttle = doubleTostring(msg->throttle_pedal_actual);
}

// gps数据处理（发送）
void AppClient::GpsCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
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
void AppClient::modeCallback(const std_msgs::Int32::ConstPtr &msg)
{
    int32_t mode = msg->data;
    mode_order = doubleTostring(double(mode));
}

// 车辆是否处于自驾模式 
void AppClient::autoCallback(const std_msgs::Int32::ConstPtr &msg) {
    int32_t is_auto = msg->data;
    auto_mode = to_string(is_auto);
	last_pub_autodrive_time = ros::Time::now().toSec();
}

// scenario_manager中的终点队列是否为空
void AppClient::goalCallback(const std_msgs::Int32::ConstPtr &msg) {
    int32_t is_goal_empty = msg->data;
    goal_status = to_string(is_goal_empty);
	last_pub_goalstatus_time = ros::Time::now().toSec();
}

// 底盘数据的发送
void AppClient::SendDebugInfo()
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
                    mode_order + "," + nn + "," +
					auto_mode + "," + nn + "," +
                    goal_status + "," + nn;
	json_temp_3["content"] = content;
	// printf("pub info %s \n", content.c_str());
	SendBuf3 = write.write(json_temp_3);
	send(client_flag, SendBuf3.c_str(), SendBuf3.size(), 0);

	// 为了避免因为is_auto_driving发送频率不够, 设置的一个时延。只有当1s以上没有收到is_auto_driving的消息时，才会将auto_mode置为null
	if (ros::Time::now().toSec() - last_pub_autodrive_time > 1)
		auto_mode = "null";

	counter++;
	if (counter == 8)
		counter = 0;
}


void AppClient::control(int &sig)
{
	if (sig == 0)  // 杀死所有终端和进程，并重启一个app节点
	{
        FILE *fp;
		fp=popen("~/autoware.gf2/scripts/app/clear.sh", "r"); 
		pclose(fp);
	}

	if (sig == 1) {  // 发布终点
		// popen()是一个用于创建进程并建立管道通信的函数
		FILE *fp3 = popen("~/autoware.gf2/scripts/app/pub_goal.sh", "r");  // "r"：表示从子进程读取输出。
		pclose(fp3);
	}

	if (sig == 2) {  // 将发布终点的节点关闭，便于下次打开时重新发布终点
		// FILE *fp4 = popen("rosnode kill /pub_goal ", "r");
		FILE *fp4 = popen("ps -ef | grep pub_goal | cut -c 9-15 | xargs kill -s 9 ", "r");
		pclose(fp4);
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
		// FILE *fp3 = popen("rostopic pub -1 /ar_track_alvar/charge_status std_msgs/Int16 -- '1' ");
	}
	if (sig == 21)  // 正常模式（可以切换贴边、巡检）启动
	{
		FILE *fp;
		// char buffer[80];
		fp=popen("~/autoware.gf2/scripts/app/normal.sh", "r");    //以读方式，fork产生一个子进程，执行shell命令
		// fgets(buffer,sizeof(buffer), fp);       //读取shell脚本中输出(stdout)的值
		// printf("%s", buffer);
		pclose(fp);

		// 扫盘开启指令
		// cleaning_cmd.vehicle_status.data = 0;
		// cleaning_cmd.cleaning_status.is_cleaning.data = true;
		// pub_cleaning_cmd.publish(cleaning_cmd);
	}
	if (sig == 20)  // 正常模式关闭
	{
		FILE *fp;
		fp=popen("~/autoware.gf2/scripts/app/stop_normal.sh", "r"); 
		pclose(fp);
	}
	if (sig == 121)
	{
		// FILE *fp;
		// fp=popen("~/autoware.gf2/scripts/app/cover.sh", "r"); 
		// pclose(fp);
	}
	if (sig == 120)
	{
		// FILE *fp;
		// fp=popen("~/autoware.gf2/scripts/app/stop_cover.sh", "r");
		// pclose(fp);
	}
    if (sig == 22)  // 驱动启动
	{
		FILE *fp;
		fp=popen("~/autoware.gf2/scripts/app/driver.sh", "r");
		pclose(fp);
	}
	if (sig == 23)  // 驱动关闭
	{
		FILE *fp;
		fp=popen("~/autoware.gf2/scripts/app/stop_driver.sh", "r");
		pclose(fp);
	}
    if (sig == 24)
	{
		// FILE *fp;
		// fp=popen("~/autoware.gf2/scripts/app/reverse.sh", "r");   
		// pclose(fp);
	}
	if (sig == 25)
	{
		// FILE *fp;
		// fp=popen("~/autoware.gf2/scripts/app/stop_reverse.sh", "r");   
		// pclose(fp); 
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

	if (vehicle_mode  == 1)  // 如果进入遥控模式，可以用油门、刹车、转向
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
	ros::init(argc, argv, "app_client");

	AppClient myclient;
	ros::Rate loop_rate(100);

	// 创建一个子线程，用于接收来自app的控制指令
	std::thread recv_thread(&AppClient::recvLoop, &myclient);

	while (ros::ok())
	{
		ros::spinOnce();
		myclient.SendDebugInfo();
		loop_rate.sleep();
	}
	cout << "Car's client for app is closing" << endl;
	shutdown(myclient.client_flag, SHUT_RDWR);
	close(myclient.client_flag);
	cout << "client closed !" << endl;

	recv_thread.join();
	return 0;
}
