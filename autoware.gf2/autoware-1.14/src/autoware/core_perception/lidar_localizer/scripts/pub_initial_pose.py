#! /usr/bin/env python
# coding=UTF-8

import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped

class PubInitialGoal:
    def __init__(self):
        
        self.initial_goal = PoseWithCovarianceStamped()
        self.initial_goal.header.frame_id = "world"
        self.initial_goal.header.stamp = rospy.Time.now()
        self.initial_goal.pose.pose.position.x = -22.3897666931 
        self.initial_goal.pose.pose.position.y = -238.551651001
        self.initial_goal.pose.pose.position.z = 0.0
        self.initial_goal.pose.pose.orientation.x = 0.0
        self.initial_goal.pose.pose.orientation.y = 0.0
        self.initial_goal.pose.pose.orientation.z = 0.620511263802  
        self.initial_goal.pose.pose.orientation.w = 0.784197533466


        self.pub_initial_goal_ = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

    def pubInit(self):
        self.pub_initial_goal_.publish(self.initial_goal)



if __name__ == "__main__":
    rospy.init_node("pub_initial_pose", anonymous=False)
    pub_goals = PubInitialGoal()
    time.sleep(2)
    # 先发布 垃圾桶的位置
    pub_goals.pubInit()
    print("pub initial pose!")
    # 阻塞程序结束
    rospy.spin()