#include "ros/ros.h"
#include "std_msgs/Bool.h"
// include- msgs header file
// Example: #include "pix_driver_msgs/BrakeCommand.h"
// #include pix_driver_msgs/protocols["name"].h
#include "pix_driver_msgs/acu_sweepctrlcmd_107.h"

ros::Publisher pub_sweeper_status;
bool sweeper_flag = false;


void timer_callback(const ros::TimerEvent &te){
    std_msgs::Bool sweeper_status_msg;
    sweeper_status_msg.data = sweeper_flag;
    pub_sweeper_status.publish(sweeper_status_msg);  
}

void reversed_callback(const ros::TimerEvent &te){
    sweeper_flag = !sweeper_flag;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pix_driver_test");
    ros::NodeHandle nh;
    ros::Timer timer;
    ros::Timer timer1;
    timer = nh.createTimer(ros::Duration(1/50.0), timer_callback); 
    timer1 = nh.createTimer(ros::Duration(8.0), reversed_callback); 
    pub_sweeper_status = 
        nh.advertise<std_msgs::Bool>("/sweeper_status", 10, true); 
    ros::spin();

    return 0;
}