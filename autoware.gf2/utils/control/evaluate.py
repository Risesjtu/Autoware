import rospy
from autoware_msgs.msg import LaneArray
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import os
import math


class MPCEvaluate(object):
    def __init__(self):
        self.global_path = LaneArray()
        self.mpc_debug = Float64MultiArray()
        self.current_pose = PoseStamped()
        self.lat_err = []
        self.heading_err = []
        self.actual_traj = []
        
        self.sub_global_path = rospy.Subscriber("lane_waypoints_array", LaneArray, self.GlobalPathCallback)
        self.sub_mpc_debug = rospy.Subscriber("/mpc_follower/debug/debug_values", Float64MultiArray, self.MPCDebugCallback)
        self.sub_current_pose = rospy.Subscriber("current_pose", PoseStamped, self.CurrentPoseCallback)
        home_dir = os.path.expanduser("~")
        self.path = home_dir + "/.autoware/sjtu_data/mpc_debug"
        print(self.path)
        if not os.path.exists(self.path):
            os.mkdir(self.path)
        
        self.data_path = self.path + "/data"
        if not os.path.exists(self.data_path):
            os.mkdir(self.data_path)
        self.i = 0
        while True:
            if os.path.exists(self.data_path + '/%s' % self.i):
                self.i += 1 
                continue
            else:
                os.mkdir(self.data_path + '/%s' % self.i)
                break
        
        self.first_point_debug = True
        self.first_point_traj = True
        self.dist = 0.0
        self.interval = 0.2
        
    
    def GlobalPathCallback(self, msg):
        self.global_path = msg
        save_path = self.data_path + '/%s' % self.i
        waypoints = self.global_path.lanes[0].waypoints
        with open(save_path + "/global_path.txt", 'w') as f:
            x = []
            y = []
            for i in range(len(waypoints)):
                x.append(waypoints[i].pose.pose.position.x)
                y.append(waypoints[i].pose.pose.position.y)
            f.writelines(str(x))
            f.write('\n')
            f.writelines(str(y))
        print("receive global path!")
        
    def MPCDebugCallback(self, msg):
        # first_point = True
        self.mpc_debug = msg
        # dist = 0.0
        if len(self.actual_traj) >= 2:
            self.dist = math.sqrt((self.actual_traj[-1].x - self.actual_traj[-2].x)**2 + (self.actual_traj[-1].y - self.actual_traj[-2].y)**2)
        else:
            self.dist = 0.0

        self.lat_err.append(self.mpc_debug.data[5])
        self.heading_err.append(self.mpc_debug.data[8])
        save_path = self.data_path + '/%s' % self.i
        with open(save_path + "/lateral_error.txt", 'w') as f:
            f.writelines(str(self.lat_err))
        with open(save_path + "/heading_error.txt", 'w') as f:
            f.writelines(str(self.heading_err))
        print("reveive mpc debug msg!")
        
    def CurrentPoseCallback(self, msg):
        self.current_pose = msg
        if len(self.actual_traj) >= 2:
            self.dist = math.sqrt((self.actual_traj[-1].x - self.actual_traj[-2].x)**2 + (self.actual_traj[-1].y - self.actual_traj[-2].y)**2)
        save_path = self.data_path + '/%s' % self.i
        self.actual_traj.append(self.current_pose.pose.position)
        with open(save_path + "/actual_traj.txt", 'w') as f:
            actual_x = []
            actual_y = []
            for i in range(len(self.actual_traj)):
                actual_x.append(self.actual_traj[i].x)
                actual_y.append(self.actual_traj[i].y)
            f.writelines(str(actual_x))
            f.write('\n')
            f.writelines(str(actual_y))  
        print("receive current_pose!")
        
        
    # def __del__(self):
    #     print("into deleting!")
    #     plt.subplot(311)
    #     global_path_x = []
    #     global_path_y = []
    #     actual_path_x = []
    #     actual_path_y = []
    #     waypoints = self.global_path.lanes[0].waypoints
    #     for i in range(len(waypoints)):
    #         global_path_x.append(waypoints[i].pose.pose.position.x)
    #         global_path_y.append(waypoints[i].pose.pose.position.y)
    #     for i in range(len(self.actual_traj)):
    #         actual_path_x.append(self.actual_traj[i].x)
    #         actual_path_y.append(self.actual_traj[i].y)
    #     plt.plot(global_path_x, global_path_y, label="global_path")
    #     plt.plot(actual_path_x, actual_path_y, label="actual_path")
    #     plt.legend()
    #     plt.subplot(312)
    #     length = len(self.lat_err)
    #     x = range(length)
    #     plt.plot(x, self.lat_err, label="lateral error")
    #     plt.legend()
    #     plt.subplot(313)
    #     length = len(self.heading_err)
    #     x = range(length)
    #     plt.plot(x, self.heading_err, label="heading error")
    #     plt.legend()
        
    #     i = 0
    #     while True:
    #         if os.path.exists(self.path + '/%s.png' % i):
    #             i += 1 
    #             continue
    #         else:
    #             plt.savefig(self.path + '/%s.png' % i)
    #             break
        
            
           
if __name__ == '__main__':
    rospy.init_node('MPC_evaluate', anonymous=True)
    MPC_evaluator = MPCEvaluate()
    rospy.spin()