#!/usr/bin/env python
#
# file: $ISIP_EXP/tuh_dpath/exp_0074/scripts/decode.py
#
# revision history:
#  20190925 (TE): first version
#
# usage:
#  python decode.py odir mfile data
#
# arguments:
#  odir: the directory where the hypotheses will be stored
#  mfile: input model file
#  data: the input data list to be decoded
#
# This script decodes data using a simple MLP model.
#------------------------------------------------------------------------------

# import modules
#
import sys
import os

# ros:
import rospy
import numpy as np 

# custom define messages:
from sensor_msgs.msg import LaserScan
from barn_msgs.msg import BARN_data
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from stable_baselines3 import PPO
from custom_cnn_full import *

#-----------------------------------------------------------------------------
#
# global variables are listed here
#
#-----------------------------------------------------------------------------
#set_seed(SEED1)

# for reproducibility, we seed the rng
#       
policy_kwargs = dict(
    features_extractor_class=CustomCNN,
    features_extractor_kwargs=dict(features_dim=256),
)

#------------------------------------------------------------------------------
#
# the main program starts here
#
#------------------------------------------------------------------------------
class DrlInference:
    # Constructor
    def __init__(self, model=None):
        # initialize data:  
        self.LASER_CLIP = 10
        self.scan = [] 
        self.goal = []
        self.vx = 0
        self.wz = 0
        self.model = None

        # jackal robot distance:
        delta = 0.375*np.pi/180
        b_l = 0.165
        b_f = 0.145
        b_r = 0.165
        theta1 = np.linspace(120, 0, num=121)*delta
        theta2 = np.linspace(1, 109, num=109)*delta
        theta3 = np.linspace(129, 0, num=130)*delta
        theta4 = np.linspace(1, 130, num=130)*delta
        theta5 = np.linspace(109, 1, num=109)*delta
        theta6 = np.linspace(0, 120, num=121)*delta
        robot_c1 = b_l/np.cos(theta1)
        robot_c2 = b_l/np.cos(theta2)
        robot_c3 = b_f/np.cos(theta3)
        robot_c4 = b_f/np.cos(theta4)
        robot_c5 = b_r/np.cos(theta5)
        robot_c6 = b_r/np.cos(theta6)
        self.jackal_dis = np.concatenate((robot_c1, robot_c2, robot_c3, robot_c4, robot_c5, robot_c6), axis=None)

        # turtlebot robot distance:
        robot_l = np.linspace(0.1*np.sqrt(3), 0.1, num=361)
        robot_r = np.linspace(0.10020335, 0.1*np.sqrt(3), num=359) 
        self.turtlebot_dis = np.concatenate((robot_l, robot_r), axis=None)
        
        # parameters:
        self.start = rospy.get_param('~start', False)
    
        # load model:
        if(model == None):
            model_file = rospy.get_param('~model_file', "./model/drl_vo.zip")
            self.model = PPO.load(model_file, device="cpu")
        else:
            self.model = model
        print("Finish loading model.")

        # initialize ROS objects
        self.barn_data_sub = rospy.Subscriber("barn_data", BARN_data, self.barn_data_callback, queue_size=1, buff_size=2**24)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1, latch=False)
        
    # Callback function for the barn_data subscriber
    def barn_data_callback(self, barn_data_msg):
        self.scan = barn_data_msg.scan
        self.goal = barn_data_msg.goal_cart
        cmd_vel = Twist()

        if(self.start): # start navigation  
            # minimum distance:
            scan = np.array(self.scan[-720+240:-240])
            scan = scan[scan!=0]
            if(scan.size!=0):
                min_scan_dist = np.amin(scan)
            else:
                min_scan_dist = 10
            
            # if the goal is close to the robot:
            if(np.linalg.norm(self.goal) <= 0.9):  # goal margin
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
            elif(min_scan_dist <= 0.45): # obstacle margin
                cmd_vel.linear.x = 0 
                cmd_vel.angular.z = 0.7
            else:
                # purepursit cmd:
                (v_pp, w_pp) = self.calculate_velocity(np.array([self.goal[0], self.goal[1]]))

                # ped_map:
                ped_pos = np.zeros(12800)

                # scan_map:
                laser_scan = np.array(self.scan, dtype=np.float32)
                laser_scan[laser_scan > self.LASER_CLIP] = self.LASER_CLIP
                # min-avg pooling:
                scan_avg = np.zeros((20,80))
                for n in range(10):
                    scan_tmp = laser_scan[n*720:(n+1)*720] - self.jackal_dis + self.turtlebot_dis
                    for i in range(80):
                        scan_avg[2*n, i] = np.min(scan_tmp[i*9:(i+1)*9])
                        scan_avg[2*n+1, i] = np.mean(scan_tmp[i*9:(i+1)*9])
                # stacking: 
                scan_avg = scan_avg.reshape(1600)
                scan_avg_map = np.matlib.repmat(scan_avg,1,4)
                self.scan = scan_avg_map.reshape(6400)
                # MaxAbsScaler:
                s_min = 0
                s_max = self.LASER_CLIP
                self.scan = 2 * (self.scan - s_min) / (s_max - s_min) + (-1)

                # goal:
                # MaxAbsScaler:
                g_min = -1
                g_max = 1
                goal_orignal = np.array(self.goal, dtype=np.float32)
                self.goal = 2 * (goal_orignal - g_min) / (g_max - g_min) + (-1)

                # observation:
                self.observation = np.concatenate((ped_pos, self.scan, self.goal), axis=None) 

                # drl-vo infrence: calculate the goal velocity of the robot and send the command
                action, _states = self.model.predict(self.observation, deterministic=True)

                # velocities:
                vx_min = 0
                if(min_scan_dist >= 2.2): # free space margin
                    vx_max = 0.5 #2
                else:
                    vx_max = 0.5 
                vz_min = -0.7
                vz_max = 0.7

                # MaxAbsScaler inverse:
                v_drl = (action[0] + 1) * (vx_max - vx_min) / 2 + vx_min
                w_drl = (action[1] + 1) * (vz_max - vz_min) / 2 + vz_min

                cmd_vel.linear.x = 0.6*v_drl + 0.4*v_pp
                cmd_vel.angular.z = 0.8*w_drl + 0.2*w_pp
        
        # publish the cmd_vel:
        if not np.isnan(cmd_vel.linear.x) and not np.isnan(cmd_vel.angular.z): # ensure data is valid
            self.cmd_vel_pub.publish(cmd_vel)

    def calculate_velocity(self, goal):
        # calculate the radius of curvature
        R = np.dot(goal, goal) / (2. * goal[1] + 1e-8)
        w_max = 2
        v_max = 0.5
        v_cmd = w_cmd = 0.

        if(R == 0):
            v_cmd = 0.
            w_cmd = w_max / np.sign(R)
        elif(R >= 1e8):
            v_cmd = v_max
            w_cmd = 0.0
        else:
            v_cmd = np.sign(goal[0]) * 1
            w_cmd = v_cmd / R
            
        r = 0.098 #self.wheel_radius
        L = 0.262 #self.wheel_base
        u = v_cmd / r + L * w_cmd / (2. * r) * np.array([-1, 1])
        u_limit = min(v_max, w_max * L) / r
        u = u * u_limit / (max(abs(u[0]), abs(u[1])) + 1e-8)
        v = r / 2. * (u[0] + u[1])
        w = r / L * (u[1] - u[0])

        return (v, w)

    #
    # end of function


# begin gracefully
#

if __name__ == '__main__':
    rospy.init_node('drl_vo_inference')
    drl_infe = DrlInference()
    rospy.spin()

# end of file
