// #include <can_msgs/Frame.h>
#include <pix_driver_msgs/CANInfo.h>
#include <pix_driver_msgs/VcuReport.h>
#include <pix_driver_msgs/ThrottleReport.h>
#include <pix_driver_msgs/BrakeReport.h>
#include <pix_driver_msgs/GearReport.h>
#include <ros/ros.h>


int driveshift_ = 0;
float speed_ = 0;
float drivepedal_ = 0;
float brakepedal_ = 0;
bool is_brake_ok_ = false;
bool is_shift_ok_ = false;
bool is_throttle_ok_ = false;
bool is_speed_ok_ = false;

void VcuReportCallBack(pix_driver_msgs::VcuReport msg)
{
    // speed_ = msg.speed;
    is_speed_ok_ = true;
}

void ThrottleCallBack(pix_driver_msgs::ThrottleReport msg)
{
    drivepedal_ = msg.throttle_pedal_actual;
    is_throttle_ok_ = true;

}
void BrakeCallBack(pix_driver_msgs::BrakeReport msg)
{
    brakepedal_ = msg.brake_pedal_actual;
    is_brake_ok_ = true;
}
void GearCallBack(pix_driver_msgs::GearReport msg)
{
    if (msg.gear_actual==0)
        {driveshift_= -1;
        std::cout<<"driveshift status is error!"<<std::endl;}
    else if(msg.gear_actual==1)
        {is_shift_ok_ = true;
        driveshift_ = 128;}
    else if(msg.gear_actual==2)
        {driveshift_ = 64;
        is_shift_ok_ = true;}
    else if (msg.gear_actual==3)
        {driveshift_ = 32;
        is_shift_ok_ = true;}
    else if(msg.gear_actual==4)
       {driveshift_ = 16;
        is_shift_ok_ = true;}
    else{
        std::cout<<"driveshift status is error!"<<std::endl;}
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "caninfo");

  ros::NodeHandle nh("");

//   std::string can_device;
//   nh_param.param<std::string>("can_device", can_device, "can0");

  ros::Subscriber Vcusub = nh.subscribe("/pix/vcu_report", 10, VcuReportCallBack);
  ros::Subscriber Throttlesub = nh.subscribe("/pix/throttle_report", 10, ThrottleCallBack);
  ros::Subscriber Brakesub = nh.subscribe("/pix/brake_report", 10, BrakeCallBack);
  ros::Subscriber Gearsub = nh.subscribe("/pix/gear_report", 10, GearCallBack);
  ros::Publisher can_info_pub = nh.advertise<pix_driver_msgs::CANInfo>("can_info", 10);
//   can::ThreadedSocketCANInterfaceSharedPtr driver = std::make_shared<can::ThreadedSocketCANInterface> ();

  // initialize device at can_device, 0 for no loopback.
  ros::Rate r(10);
  while (ros::ok())
  {
    // can_msg
    if(is_brake_ok_&&is_shift_ok_&&is_throttle_ok_&&is_speed_ok_)
    {
        is_brake_ok_ = false;
        is_shift_ok_ = false;
        is_throttle_ok_ = false;
        is_speed_ok_ = false;
        pix_driver_msgs::CANInfo can_msg;
        can_msg.devmode = 1;
        can_msg.strmode = 1;
        can_msg.driveshift = driveshift_;
        can_msg.speed = speed_;
        can_msg.drivepedal = drivepedal_;
        can_msg.brakepedal = brakepedal_;
        can_msg.light = 0;
        can_info_pub.publish(can_msg);
        std::cout<<"CAN INFO publish successfully!"<<std::endl;
    }
    else
    {
        std::cout<<"some status is error"<<std::endl;
        std::cout<<"brake is "<< is_brake_ok_ <<std::endl;
        std::cout<<"throttle is "<<is_throttle_ok_ <<std::endl;
        std::cout<<"speed is "<< is_speed_ok_ <<std::endl;
        std::cout<<"shift is "<< is_shift_ok_ <<std::endl;
    }
    ros::spinOnce();
    r.sleep();
  }
  
  
//   return 0
}