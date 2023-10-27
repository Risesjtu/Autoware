#! /usr/bin/env python
# coding=UTF-8

import rospy
import time
from autoware_msgs.msg import DockInfo

class PubGoal:
    def __init__(self):
        
        self.charge_goal = DockInfo()
        self.charge_goal.task = "charge"
        self.charge_goal.goal.header.frame_id = "world"
        self.charge_goal.goal.header.stamp = rospy.Time.now()
        self.charge_goal.goal.pose.position.x = 19.0      
        self.charge_goal.goal.pose.position.y = 1.73      
        self.charge_goal.goal.pose.position.z = 0.0
        self.charge_goal.goal.pose.orientation.x = 0.0
        self.charge_goal.goal.pose.orientation.y = 0.0
        self.charge_goal.goal.pose.orientation.z = 0.845
        self.charge_goal.goal.pose.orientation.w = 0.534  
        
        # 四号楼门口
        self.garbage_goal = DockInfo()
        self.garbage_goal.task = "garbage"
        self.garbage_goal.goal.header.frame_id = "world"
        self.garbage_goal.goal.header.stamp = rospy.Time.now()
        self.garbage_goal.goal.pose.position.x = -95.5341491699   
        self.garbage_goal.goal.pose.position.y = -192.51550293      
        self.garbage_goal.goal.pose.position.z = 0.0
        self.garbage_goal.goal.pose.orientation.x = 0.0
        self.garbage_goal.goal.pose.orientation.y = 0.0
        self.garbage_goal.goal.pose.orientation.z = 0.910279054207
        self.garbage_goal.goal.pose.orientation.w = 0.413995221556     

        # 小圆环终点
        # self.garbage_goal = DockInfo()
        # self.garbage_goal.task = "garbage"
        # self.garbage_goal.goal.header.frame_id = "world"
        # self.garbage_goal.goal.header.stamp = rospy.Time.now()
        # self.garbage_goal.goal.pose.position.x = -3.47    
        # self.garbage_goal.goal.pose.position.y = -5.18         
        # self.garbage_goal.goal.pose.position.z = 0.0
        # self.garbage_goal.goal.pose.orientation.x = 0.0
        # self.garbage_goal.goal.pose.orientation.y = 0.0
        # self.garbage_goal.goal.pose.orientation.z = 0.826
        # self.garbage_goal.goal.pose.orientation.w = 0.563   


        # 机动门口
        # self.garbage_goal = DockInfo()
        # self.garbage_goal.task = "garbage"
        # self.garbage_goal.goal.header.frame_id = "world"
        # self.garbage_goal.goal.header.stamp = rospy.Time.now()
        # self.garbage_goal.goal.pose.position.x = 198.653579712   
        # self.garbage_goal.goal.pose.position.y = 119.851844788        
        # self.garbage_goal.goal.pose.position.z = 0.0
        # self.garbage_goal.goal.pose.orientation.x = 0.0
        # self.garbage_goal.goal.pose.orientation.y = 0.0
        # self.garbage_goal.goal.pose.orientation.z = -0.466609367797
        # self.garbage_goal.goal.pose.orientation.w = 0.884463508509         

        self.pub_goal_ = rospy.Publisher('move_base_simple/goal', DockInfo, queue_size=1)

    def pubGarbage(self):
        self.pub_goal_.publish(self.garbage_goal)

    def pubCharge(self):
        self.pub_goal_.publish(self.charge_goal)


if __name__ == "__main__":
    rospy.init_node("pub_goal", anonymous=False)
    pub_goals = PubGoal()
    time.sleep(3)
    # 先发布 垃圾桶的位置
    pub_goals.pubGarbage()
    print("pub garbage!")
    # 间隔2s后，再发布充电桩的位置
    # time.sleep(2)
    # pub_goals.pubCharge()
    # print("pub charge!")
    # 阻塞程序结束
    rospy.spin()