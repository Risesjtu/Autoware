#! /usr/bin/env python
# coding=UTF-8

import rospy
from autoware_msgs.msg import DockInfo
import time
import copy

class PubGoal:
  def __init__(self):
    self.charge_goal = DockInfo()
    self.charge_goal.task = 'charge'
    self.charge_goal.goal.header.frame_id = "world"
    self.charge_goal.goal.pose.position.x = -23.0394802094
    self.charge_goal.goal.pose.position.y = -238.712463379
    self.charge_goal.goal.pose.position.z = 0.0
    self.charge_goal.goal.pose.orientation.x = 0.0
    self.charge_goal.goal.pose.orientation.y = 0.0
    self.charge_goal.goal.pose.orientation.z = 0.618625920793
    self.charge_goal.goal.pose.orientation.w = 0.785685668778


    self.garbage_goal = copy.deepcopy(self.charge_goal)
    self.garbage_goal.task = "garbage"


    self.pub_goal_ = rospy.Publisher('move_base_simple/goal',DockInfo,queue_size=1)

  def pubCharge(self):
    self.pub_goal_.publish(self.charge_goal)

  def pubGarbage(self):
    self.pub_goal_.publish(self.garbage_goal)


if __name__=="__main__":
  rospy.init_node("pub_goal_sim",anonymous=True)
  test = PubGoal()
  time.sleep(3)
  # print("pub charge!")
  # test.pubCharge()
  # time.sleep(2)
  print("pub garbage!")
  test.pubGarbage()
  rospy.spin()
