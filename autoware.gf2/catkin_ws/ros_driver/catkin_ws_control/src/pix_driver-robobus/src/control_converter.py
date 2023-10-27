#!/usr/bin/env python
#coding=utf-8

import rospy
from pix_driver_msgs.msg import GearCommand, ThrottleCommand, BrakeCommand, SteeringCommand, VehicleModeCommand, GearReport
from autoware_vehicle_msgs.msg import RawVehicleCommand

class ControlConverter:
    def __init__(self):
        self.throttle = 0.0
        self.brake = 0.0
        self.steer = 0.0
        self.gear = 0
        
        self.gear_state = 0
        
        self.sub_raw_command = rospy.Subscriber('/vehicle/raw_vehicle_cmd', RawVehicleCommand, self.raw_command_callback)
        self.sub_gear_report = rospy.Subscriber('/pix/gear_report', GearReport, self.gear_report_callback)


        self.pub_throttle = rospy.Publisher('/pix/throttle_command', ThrottleCommand, queue_size=10)
        self.pub_brake = rospy.Publisher('/pix/brake_command', BrakeCommand, queue_size=10)
        self.pub_steer = rospy.Publisher('/pix/steer_command', SteeringCommand, queue_size=10)
        self.pub_gear = rospy.Publisher('/pix/gear_command', GearCommand, queue_size=10)
        self.pub_vehicle = rospy.Publisher('/pix/vehicle_command', VehicleModeCommand, queue_size=10)

        self.throttle_msg = ThrottleCommand()
        self.brake_msg = BrakeCommand()
        self.steer_msg = SteeringCommand()
        self.gear_msg = GearCommand()
        self.vehicle_msg = VehicleModeCommand()



    def gear_report_callback(self, msg):
        self.gear_state = msg.gear_actual
    
    def raw_command_callback(self, msg):
        self.throttle = msg.control.throttle
        self.brake = msg.control.brake
        self.steer = msg.control.steering_angle
        self.gear = msg.shift.data
        stamp = rospy.Time.now()

        # AEB 和 刹车功能设计
        self.brake_msg.aeb_en_ctrl = rospy.get_param("aeb_en_ctrl", 0)
        self.brake_msg.header.stamp = stamp
        self.brake_msg.brake_pedal_target = self.brake*100.0
        self.brake_msg.brake_en_ctrl = 1
        self.pub_brake.publish(self.brake_msg)

        self.steer_msg.header.stamp = stamp
        self.steer_msg.steer_en_ctrl = 1
        # self.steer_msg.steer_angle_target = self.steer*180
        self.steer_msg.steer_angle_target = self.steer*10.0
        self.pub_steer.publish(self.steer_msg)

        self.gear_msg.header.stamp = stamp
        self.gear_msg.gear_en_ctrl = 1
        if(self.gear==0):
            self.gear_msg.gear_target = 0
        elif(self.gear==1):
            self.gear_msg.gear_target = 1
        elif(self.gear==2):
            self.gear_msg.gear_target = 2
        elif(self.gear==3):
            self.gear_msg.gear_target = 3
        elif(self.gear==4):
            self.gear_msg.gear_target = 4
        self.pub_gear.publish(self.gear_msg)

        self.throttle_msg.header.stamp = stamp
        self.throttle_msg.throttle_pedal_target = self.throttle*100.0
        self.throttle_msg.throttle_en_ctrl = 1
        self.pub_throttle.publish(self.throttle_msg)

        if(self.gear_state!=self.gear):
            self.throttle_msg.throttle_pedal_target = 0

        self.vehicle_msg.header.stamp = stamp
        self.vehicle_msg.drive_mode_ctrl = 0
        self.vehicle_msg.steer_mode_ctrl = 1
        self.vehicle_msg.vin_req = 1
        self.pub_vehicle.publish(self.vehicle_msg)



if __name__ == '__main__':
    
    rospy.init_node('autoware_pix_control_converter', anonymous=True)
    converter = ControlConverter()
    rospy.spin()