#include "ros/ros.h"

#include "can_msgs/Frame.h"

#include "pix_driver_msgs/BrakeCommand.h"
#include "pix_driver_msgs/GearCommand.h"
#include "pix_driver_msgs/ParkCommand.h"
#include "pix_driver_msgs/SteeringCommand.h"
#include "pix_driver_msgs/ThrottleCommand.h"
#include "pix_driver_msgs/VehicleModeCommand.h"

#include "brake_command_101.hpp"
#include "gear_command_103.hpp"
#include "park_command_104.hpp"
#include "steering_command_102.hpp"
#include "throttle_command_100.hpp"
#include "vehicle_mode_command_105.hpp"

int time_diff = 200000;

static ros::Publisher pub_can;

static can_msgs::Frame can_brake;
static can_msgs::Frame can_gear;
static can_msgs::Frame can_park;
static can_msgs::Frame can_steer;
static can_msgs::Frame can_throttle;
static can_msgs::Frame can_vehicle;

static pix_driver_msgs::BrakeCommand brake_command_msg;
static pix_driver_msgs::GearCommand gear_command_msg;
static pix_driver_msgs::ParkCommand park_command_msg;
static pix_driver_msgs::SteeringCommand steer_command_msg;
static pix_driver_msgs::ThrottleCommand throttle_command_msg;
static pix_driver_msgs::VehicleModeCommand vehicle_command_msg;

static Brakecommand101 brake_command;
static Gearcommand103 gear_command;
static Parkcommand104 park_command;
static Steeringcommand102 steer_command;
static Throttlecommand100 throttle_command;
static Vehiclemodecommand105 vehicle_command;

static bool brake_enable, gear_enable, park_enable, steer_enable, throttle_enable;
static int brake_prev_t=0, gear_prev_t=0, park_prev_t=0, steer_prev_t=0, throttle_prev_t=0;


static void brake_callback(const pix_driver_msgs::BrakeCommand &msg)
{
    brake_command.Reset();
    brake_command_msg = msg;
    can_brake.header.stamp = ros::Time::now();
    can_brake.dlc = 8;
    brake_command.UpdateData(
        brake_command_msg.aeb_en_ctrl,
        brake_command_msg.brake_dec,
        brake_command_msg.checksum_101, 
        brake_command_msg.brake_pedal_target,
        brake_command_msg.brake_en_ctrl
    );
    can_brake.id = brake_command.ID;
    can_brake.is_extended = false;
    uint8_t *A;
    A = brake_command.get_data();
    
    for(uint i=0;i<8;i++)
    {
        can_brake.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_brake.header.stamp.toNSec();
    brake_prev_t = t_nsec;
}

static void gear_callback(const pix_driver_msgs::GearCommand &msg)
{
    gear_command.Reset();
    gear_command_msg = msg;
    can_gear.header.stamp = ros::Time::now();
    can_gear.dlc = 8;
    gear_command.UpdateData(
        gear_command_msg.gear_target,
        gear_command_msg.gear_en_ctrl,
        gear_command_msg.checksum_103
    );
    can_gear.id = gear_command.ID;
    can_gear.is_extended= false;
    uint8_t *A;
    A = gear_command.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_gear.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_gear.header.stamp.toNSec();
    gear_prev_t = t_nsec;
}

static void park_callback(const pix_driver_msgs::ParkCommand &msg)
{
    park_command.Reset();
    park_command_msg = msg;
    can_park.header.stamp = ros::Time::now();
    can_park.dlc = 8;
    park_command.UpdateData(
        park_command_msg.checksum_104,
        park_command_msg.park_target,
        park_command_msg.park_en_ctrl
    );
    can_park.id = park_command.ID;
    can_park.is_extended = false;
    uint8_t *A;
    A = park_command.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_park.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_park.header.stamp.toNSec();
    park_prev_t = t_nsec;
}

static void steer_callback(const pix_driver_msgs::SteeringCommand &msg)
{
    steer_command.Reset();
    steer_command_msg = msg;
    can_steer.header.stamp = ros::Time::now();
    can_steer.dlc = 8;
    steer_command.UpdateData(
        steer_command_msg.steer_en_ctrl,
        steer_command_msg.steer_angle_target,
        steer_command_msg.steer_angle_spd,
        steer_command_msg.checksum_102
    );
    can_steer.id = steer_command.ID;
    can_steer.is_extended = false;
    uint8_t *A;
    A = steer_command.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_steer.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_steer.header.stamp.toNSec();
    steer_prev_t = t_nsec;
}

static void throttle_callback(const pix_driver_msgs::ThrottleCommand &msg)
{
    throttle_command.Reset();
    throttle_command_msg = msg;
    can_throttle.header.stamp = ros::Time::now();
    can_throttle.dlc = 8;
    throttle_command.UpdateData(
        throttle_command_msg.vel_target,
        throttle_command_msg.throttle_acc,
        throttle_command_msg.checksum_100,
        throttle_command_msg.throttle_pedal_target,
        throttle_command_msg.throttle_en_ctrl
    );
    can_throttle.id = throttle_command.ID;
    can_throttle.is_extended = false;
    uint8_t *A;
    A = throttle_command.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_throttle.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_throttle.header.stamp.toNSec();
    throttle_prev_t = t_nsec;
}

static void vehicle_callback(const pix_driver_msgs::VehicleModeCommand &msg)
{
    vehicle_command.Reset();
    vehicle_command_msg = msg;
    can_vehicle.header.stamp = ros::Time::now();
    can_vehicle.dlc = 8;
    vehicle_command.UpdateData(
        vehicle_command_msg.checksum_105,
        vehicle_command_msg.turn_light_ctrl,
        vehicle_command_msg.vin_req,
        vehicle_command_msg.drive_mode_ctrl,
        vehicle_command_msg.steer_mode_ctrl
    );
    can_vehicle.id = vehicle_command.ID;
    can_vehicle.is_extended = false;
    uint8_t *A;
    A = vehicle_command.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_vehicle.data[i] = *A;
        A += 1;
    }
}

void timer_callback(const ros::TimerEvent &te)
{
    int time_diff;
    int now;
    now = ros::Time::now().toNSec();
    // brake
    if(now-brake_prev_t>30000000)
    {
        for(uint i=0;i<8;i++)
        {   
            can_brake.id = brake_command.ID;
            can_brake.data[i] = 0;
        }
        pub_can.publish(can_brake);
    }
    else{
        pub_can.publish(can_brake);
    }
    // gear
    if(now-gear_prev_t>30000000)
    {
        for(uint i=0;i<8;i++)
        {   
            can_gear.id = gear_command.ID;
            can_gear.data[i] = 0;
        }
        pub_can.publish(can_gear);
    }
    else{
        pub_can.publish(can_gear);
    }
    // park
    if(now-park_prev_t>30000000)
    {
        for(uint i=0;i<8;i++)
        {
            can_park.id = park_command.ID;
            can_park.data[i] = 0;
        }
        pub_can.publish(can_park);
    }
    else{
        pub_can.publish(can_park);
    }
    // steer
    if(now-steer_prev_t>30000000)
    {
        for(uint i=0;i<8;i++)
        {
            can_steer.id = steer_command.ID;
            can_steer.data[i] = 0;
        }
        pub_can.publish(can_steer);
    }
    else{
        pub_can.publish(can_steer);
    }
    // throttle
    if(now-throttle_prev_t>30000000)
    {
        for(uint i=0;i<8;i++)
        {
            can_throttle.id = throttle_command.ID;
            can_throttle.data[i] = 0;
        }
        can_throttle.data[0] = 1;
        pub_can.publish(can_throttle);
    }
    else{
        pub_can.publish(can_throttle);
    }
    // vehicle
    pub_can.publish(can_vehicle);
    
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pix_driver_command_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_brake = nh.subscribe("/pix/brake_command", 1, brake_callback);
    ros::Subscriber sub_gear = nh.subscribe("/pix/gear_command", 1, gear_callback);
    ros::Subscriber sub_park = nh.subscribe("/pix/park_command", 1, park_callback);
    ros::Subscriber sub_steer = nh.subscribe("/pix/steer_command", 1, steer_callback);
    ros::Subscriber sub_throttle = nh.subscribe("/pix/throttle_command", 1, throttle_callback);
    ros::Subscriber sub_vehicle = nh.subscribe("/pix/vehicle_command", 1, vehicle_callback);

    pub_can = nh.advertise<can_msgs::Frame>("/sent_messages", 5, true);
    
    ros::Timer set_speed = nh.createTimer(ros::Duration(1/50.0), timer_callback);
    ros::spin();

    return 0;
}



