#!/usr/bin/env python

import rospy
from pix_driver_msgs.msg import GearCommand, ThrottleCommand, BrakeCommand, SteeringCommand,\
    VehicleModeCommand, GearReport, acu_sweepctrlcmd_107
from autoware_msgs.msg import VehicleCmd
from std_msgs.msg import Bool
import math
import time
import copy

class ControlConverter:
    def __init__(self):
        self.speed = 0.0
        self.brake = 0.0
        self.steer = 0.0
        self.gear = 0
        self.gear_act = 3
        self.driver_mode = 0
        self.pre_gear = 3
        self.sweeper_status = False
        self.pre_sweeper_status = False
        
        self.sub_control_command = rospy.Subscriber('/vehicle_cmd', VehicleCmd, self.control_command_callback)
        self.sub_gear_report = rospy.Subscriber('/pix/gear_report', GearReport, self.gear_report_callback)
        self.sub_sweeper_status = rospy.Subscriber('/sweeper_status', Bool, self.acu_callback)

        self.pub_throttle = rospy.Publisher('/pix/throttle_command', ThrottleCommand, queue_size=10)
        self.pub_brake = rospy.Publisher('/pix/brake_command', BrakeCommand, queue_size=10)
        self.pub_steer = rospy.Publisher('/pix/steer_command', SteeringCommand, queue_size=10)
        self.pub_gear = rospy.Publisher('/pix/gear_command', GearCommand, queue_size=10)
        self.pub_vehicle = rospy.Publisher('/pix/vehicle_command', VehicleModeCommand, queue_size=10)
        self.pub_acu = rospy.Publisher('/pix/acu_sweeperctrlcmd', acu_sweepctrlcmd_107, queue_size=10)

        self.throttle_msg = ThrottleCommand()
        self.brake_msg = BrakeCommand()
        self.steer_msg = SteeringCommand()
        self.gear_msg = GearCommand()
        self.vehicle_msg = VehicleModeCommand()
        self.acu_msg = acu_sweepctrlcmd_107()

    
    def control_command_callback(self, msg):
        self.speed = abs(msg.ctrl_cmd.linear_velocity)
        #self.brake = msg.ctrl_cmd.linear_acceleration 
        self.steer = -500*(msg.ctrl_cmd.steering_angle/0.43)

        stamp = rospy.Time.now()

        #if(self.gear != self.gear_act):
        #    self.speed = 0

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
        self.steer_msg.steer_angle_target = -self.steer
        self.pub_steer.publish(self.steer_msg)

        self.gear_msg.header.stamp = stamp
        self.gear_msg.gear_en_ctrl = 1
        #self.gear_msg.gear_target = self.gear
        if msg.gear_cmd.gear == 4:
            self.gear_msg.gear_target = 4
        elif msg.gear_cmd.gear == 2:
            self.gear_msg.gear_target = 2
        else:
            self.gear_msg.gear_target = 3
        
        self.pub_gear.publish(self.gear_msg)

        self.driver_mode = 1
        self.vehicle_msg.header.stamp = stamp
        self.vehicle_msg.drive_mode_ctrl = self.driver_mode
        self.vehicle_msg.steer_mode_ctrl = 1
        self.pub_vehicle.publish(self.vehicle_msg)
        
        self.acu_msg.header.stamp = stamp
        self.pub_acu.publish(self.acu_msg)
        
    def shift_callback(self, msg):
        self.gear = msg.shift.data
    
    def gear_report_callback(self, msg):
        self.gear_act = msg.gear_actual
        
    def acu_callback(self, msg):
        self.sweeper_status = msg.data
        if self.sweeper_status != self.pre_sweeper_status:
            if self.sweeper_status:
                self.acu_msg.MouthpieceUpDownCtrl = 2
                print("into sweeping!")
            else:
                self.acu_msg.MouthpieceUpDownCtrl = 1
                print("out sweeping!")
        self.pre_sweeper_status = msg.data
        

if __name__ == '__main__':
    
    rospy.init_node('autoware_pix_control_converter', anonymous=True)
    converter = ControlConverter()
    rospy.spin()
