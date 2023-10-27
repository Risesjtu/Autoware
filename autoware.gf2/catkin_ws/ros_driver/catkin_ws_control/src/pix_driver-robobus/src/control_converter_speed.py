#!/usr/bin/env python

import rospy
from pix_driver_msgs.msg import GearCommand, ThrottleCommand, BrakeCommand, SteeringCommand, VehicleModeCommand, GearReport
from autoware_control_msgs.msg import ControlCommandStamped
from autoware_vehicle_msgs.msg import ShiftStamped
import math
import time

class ControlConverter:
    def __init__(self):
        self.speed = 0.0
        self.brake = 0.0
        self.steer = 0.0
        self.gear = 0
        self.gear_act = 3
        self.driver_mode = 0
        self.pre_gear = 3
        
        self.sub_control_command = rospy.Subscriber('/control/control_cmd', ControlCommandStamped, self.control_command_callback)
        self.sub_shift_command = rospy.Subscriber('/control/shift_cmd', ShiftStamped, self.shift_callback)
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

    
    def control_command_callback(self, msg):
        self.speed = abs(msg.control.velocity)
        self.brake = msg.control.acceleration 
        self.steer = msg.control.steering_angle

        stamp = rospy.Time.now()

        if(self.gear != self.gear_act):
            self.speed = 0

        self.throttle_msg.header.stamp = stamp
        self.throttle_msg.vel_target = self.speed
        self.throttle_msg.throttle_en_ctrl = 1
        
        self.pub_throttle.publish(self.throttle_msg)

        self.brake_msg.header.stamp = stamp
        self.brake_msg.brake_pedal_target = self.brake*0
        self.brake_msg.brake_en_ctrl = 1
        self.pub_brake.publish(self.brake_msg)

        self.steer_msg.header.stamp = stamp
        self.steer_msg.steer_en_ctrl = 1
        self.steer_msg.steer_angle_target = -self.steer*180
        self.pub_steer.publish(self.steer_msg)

        self.gear_msg.header.stamp = stamp
        self.gear_msg.gear_en_ctrl = 1
        self.gear_msg.gear_target = self.gear
        
        self.pub_gear.publish(self.gear_msg)

        self.driver_mode = 1
        self.vehicle_msg.header.stamp = stamp
        self.vehicle_msg.drive_mode_ctrl = self.driver_mode
        self.pub_vehicle.publish(self.vehicle_msg)
    
    def shift_callback(self, msg):
        self.gear = msg.shift.data
    
    def gear_report_callback(self, msg):
        self.gear_act = msg.gear_actual
    




if __name__ == '__main__':
    
    rospy.init_node('autoware_pix_control_converter', anonymous=True)
    converter = ControlConverter()
    rospy.spin()