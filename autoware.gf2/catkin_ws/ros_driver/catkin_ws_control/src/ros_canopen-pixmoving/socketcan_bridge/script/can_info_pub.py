#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs
from pix_driver_msgs.msg import CANInfo
from pix_driver_msgs.msg import VcuReport
from pix_driver_msgs.msg import ThrottleReport
from pix_driver_msgs.msg import BrakeReport
from pix_driver_msgs.msg import GearReport





def ThrottleReportcallback(msg):
    global is_throttle_ok
    drivepedal_ = msg.throttle_pedal_actual
    is_throttle_ok = True
    # print("throttle!")


def BrakeReportcallback(msg):
    global is_brake_ok
    brakepedal_ = msg.brake_pedal_actual
    is_brake_ok = True


def VcuReportcallback(msg):
    global is_speed_ok
    speed_ = msg.speed
    is_speed_ok = True
    # print("vcu!")


def GearReportcallback(msg):
    global is_shift_ok
    # print("gear!")
    if msg.gear_actual==0:
        driveshift_= -1
        raise ROSInterruptException
    elif msg.gear_actual==1:
        is_shift_ok = True
        driveshift_ = 128
    elif msg.gear_actual==2:
        driveshift_ = 64
        is_shift_ok = True
    elif msg.gear_actual==3:
        driveshift_ = 32
        is_shift_ok = True
    elif msg.gear_actual==4:
        driveshift_ = 16
        is_shift_ok = True
    else:
        raise ROSInterruptException
      
 
if __name__ == '__main__':
    is_shift_ok = False
    is_speed_ok = False
    is_throttle_ok = False
    is_brake_ok = False
    driveshift_ = 0
    speed_ = 0
    drivepedal_ = 0
    brakepedal_ = 0
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/pix/vcu_report", VcuReport, VcuReportcallback)
    rospy.Subscriber("/pix/throttle_report", ThrottleReport, ThrottleReportcallback)
    rospy.Subscriber("/pix/brake_report", BrakeReport, BrakeReportcallback)
    rospy.Subscriber("/pix/gear_report", GearReport, GearReportcallback)
    pub = rospy.Publisher('can_info', CANInfo, queue_size=100)

    rate = rospy.Rate(10)
    i=0
    while not rospy.is_shutdown():
        msg = CANInfo()
        msg.tm = ""
        msg.devmode = 1
        msg.strmode = 1
        msg.driveshift = driveshift_
        msg.speed = speed_
        msg.drivepedal = drivepedal_
        msg.brakepedal = brakepedal_
        msg.light = 0
        if (is_brake_ok==True and is_shift_ok==True and is_speed_ok==True and is_throttle_ok==True):            
            i += 1
            pub.publish(msg)
            print("the {} send successfully!".format(i))
            is_ok = False
            is_shift_ok = False
            is_speed_ok = False
            is_throttle_ok = False
            is_brake_ok = False
        else:
            # print("waiting for other messages! state: barke is {}, shift is {}, speed is {}, throttle is {}".format
            # (is_brake_ok, is_shift_ok, is_speed_ok, is_throttle_ok))   
            pass    
        rate.sleep()
