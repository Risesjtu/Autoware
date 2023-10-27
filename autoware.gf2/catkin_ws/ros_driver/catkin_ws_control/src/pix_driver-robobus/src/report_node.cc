#include "ros/ros.h"
#include "can_msgs/Frame.h"

#include "pix_driver_msgs/SteerReport.h"
#include "pix_driver_msgs/GearReport.h"
#include "pix_driver_msgs/BmsReport.h"
#include "pix_driver_msgs/BrakeReport.h"
#include "pix_driver_msgs/ParkReport.h"
#include "pix_driver_msgs/ThrottleReport.h"
#include "pix_driver_msgs/VcuReport.h"
#include "std_msgs/Header.h"

#include "steering_report_502.hpp"
#include "gear_report_503.hpp"
#include "bms_report_512.hpp"
#include "brake_report_501.hpp"
#include "park_report_504.hpp"
#include "throttle_report_500.hpp"
#include "vcu_report_505.hpp"

#include <math.h>
#include <iostream>


static can_msgs::Frame can_frame_msg;

static ros::Publisher pub_steering;
static ros::Publisher pub_gear;
static ros::Publisher pub_bms;
static ros::Publisher pub_brake;
static ros::Publisher pub_park;
static ros::Publisher pub_throttle;
static ros::Publisher pub_vcu;

static pix_driver_msgs::SteerReport steer_report_msg;
static pix_driver_msgs::GearReport gear_report_msg;
static pix_driver_msgs::BmsReport bms_report_msg;
static pix_driver_msgs::BrakeReport brake_report_msg;
static pix_driver_msgs::ParkReport park_report_msg;
static pix_driver_msgs::ThrottleReport throttle_report_msg;
static pix_driver_msgs::VcuReport vcu_report_msg;

static Steeringreport502 steer_report;
static Gearreport503 gear_report;
static Bmsreport512 bms_report;
static Brakereport501 brake_report;
static Parkreport504 park_report;
static Throttlereport500 throttle_report;
static Vcureport505 vcu_report;

static void can_callback(const can_msgs::Frame &msg)
{
    can_frame_msg = msg;
    std_msgs::Header header;
    header.frame_id = "pix";
    header.stamp = can_frame_msg.header.stamp;
    if(can_frame_msg.id==steer_report.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
            byte_temp[i] = can_frame_msg.data[i];
        }
        steer_report.update_bytes(byte_temp);
        steer_report.Parse();
        steer_report_msg.header = header;
        steer_report_msg.steer_angle_actual = steer_report.steer_angle_actual;
        steer_report_msg.steer_angle_spd_actual = steer_report.steer_angle_spd_actual;
        steer_report_msg.steer_en_state = steer_report.steer_en_state;
        steer_report_msg.steer_flt1 = steer_report.steer_flt1;
        steer_report_msg.steer_flt2 = steer_report.steer_flt2;
        pub_steering.publish(steer_report_msg);
    }
    if(can_frame_msg.id==gear_report.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
            byte_temp[i] = can_frame_msg.data[i];
        }
        gear_report.update_bytes(byte_temp);
        gear_report.Parse();
        gear_report_msg.header = header;
        gear_report_msg.gear_actual = gear_report.gear_actual;
        gear_report_msg.gear_flt = gear_report.gear_flt;
        pub_gear.publish(gear_report_msg);
    }
    if(can_frame_msg.id==bms_report.ID)
    {
       uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
            byte_temp[i] = can_frame_msg.data[i];
        }
        bms_report.update_bytes(byte_temp);
        bms_report.Parse();
        bms_report_msg.header = header;
        bms_report_msg.battery_current = bms_report.battery_current;
        bms_report_msg.battery_soc = bms_report.battery_soc;
        bms_report_msg.battery_voltage = bms_report.battery_votage;
        pub_bms.publish(bms_report_msg);
    }
    if(can_frame_msg.id==brake_report.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
            byte_temp[i] = can_frame_msg.data[i];
        }
        brake_report_msg.header = header;
        brake_report.update_bytes(byte_temp);
        brake_report.Parse();
        brake_report_msg.brake_en_state = brake_report.brake_en_state;
        brake_report_msg.brake_flt1 = brake_report.brake_flt1;
        brake_report_msg.brake_flt2 = brake_report.brake_flt2;
        brake_report_msg.brake_pedal_actual = brake_report.brake_pedal_actual;
        pub_brake.publish(brake_report_msg);
    }
    if(can_frame_msg.id==park_report.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
            byte_temp[i] = can_frame_msg.data[i];
        }
        park_report_msg.header = header;
        park_report.update_bytes(byte_temp);
        park_report.Parse();
        park_report_msg.park_flt = park_report.park_flt;
        park_report_msg.parking_actual = park_report.parking_actual;
        pub_park.publish(park_report_msg);
    }
    if(can_frame_msg.id==throttle_report.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
            byte_temp[i] = can_frame_msg.data[i];
        } 
        throttle_report.update_bytes(byte_temp);
        throttle_report.Parse();
        throttle_report_msg.header = header;
        throttle_report_msg.throttle_en_state = throttle_report.throttle_en_state;
        throttle_report_msg.throttle_flt1 = throttle_report.throttle_flt1;
        throttle_report_msg.throttle_flt2 = throttle_report.throttle_flt2;
        throttle_report_msg.throttle_pedal_actual = throttle_report.throttle_pedal_actual;
        pub_throttle.publish(throttle_report_msg);
    }
    if(can_frame_msg.id==vcu_report.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
            byte_temp[i] = can_frame_msg.data[i];
        }
        vcu_report.update_bytes(byte_temp);
        vcu_report.Parse();
        vcu_report_msg.header = header;
        vcu_report_msg.acc = vcu_report.acc;
        vcu_report_msg.aeb_state = vcu_report.aeb_state;
        vcu_report_msg.backcrash_state = vcu_report.backcrash_state;
        vcu_report_msg.brake_light_actual = vcu_report.brake_light_actual;
        vcu_report_msg.chassis_errcode = vcu_report.chassis_errcode;
        vcu_report_msg.drive_mode_sts = vcu_report.drive_mode_sts;
        vcu_report_msg.frontcrash_state = vcu_report.frontcrash_state;
        vcu_report_msg.speed = vcu_report.speed;
        vcu_report_msg.steer_mode_sts = vcu_report.steer_mode_sts;
        vcu_report_msg.turn_light_actual = vcu_report.turn_light_actual;
        vcu_report_msg.vehicle_mode_state = vcu_report.vehicle_mode_state;
        pub_vcu.publish(vcu_report_msg);
    }
} 

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pix_driver_report_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/received_messages", 1, can_callback);
    pub_steering = nh.advertise<pix_driver_msgs::SteerReport>("/pix/steer_report", 1, true);
    pub_gear = nh.advertise<pix_driver_msgs::GearReport>("/pix/gear_report", 1, true);
    pub_bms = nh.advertise<pix_driver_msgs::BmsReport>("/pix/bms_report", 1, true);
    pub_brake = nh.advertise<pix_driver_msgs::BrakeReport>("/pix/brake_report", 1, true);
    pub_park = nh.advertise<pix_driver_msgs::ParkReport>("/pix/park_report", 1, true);
    pub_throttle = nh.advertise<pix_driver_msgs::ThrottleReport>("/pix/throttle_report", 1, true);
    pub_vcu = nh.advertise<pix_driver_msgs::VcuReport>("/pix/vcu_report", 1, true);
    // add another publisher

    ros::spin();
    return 0;

}
