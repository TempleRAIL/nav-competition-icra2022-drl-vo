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
from move_base import MoveBase
from enum import Enum
from nav_msgs.msg import Odometry

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
class Mode(Enum):
    INIT = 0
    STOP = 1
    TURN = 2
    DRIVE = 3
    FORWARD = 4

class DrlInference:
    # Constructor
    def __init__(self, model=None):
        # initialize data:  
        self.LASER_CLIP = 10
        self.barn_data = BARN_data()
        self.scan = np.ones(720)*self.LASER_CLIP
        self.goal = np.zeros(2)
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
        goal_x = rospy.get_param('~goal_x', 0)
        goal_y = rospy.get_param('~goal_y', 10)

        # state machine:
        self.odom = Odometry()
        self.mode = Mode.DRIVE
        self.d_f = self.LASER_CLIP
        self.d_b = self.LASER_CLIP
        self.theta = 0
        self.num_fr = 0
        self.num_fl = 0
        self.cnt = 0

        # launch move_base:
        self.goal_position = [goal_x, goal_y, 0]
        self.base_local_planner = "base_local_planner/TrajectoryPlannerROS"
        self.move_base = MoveBase(goal_position=self.goal_position, base_local_planner=self.base_local_planner)
        # make plan: 
        self.move_base.reset_robot_in_odom()
        self.move_base.make_plan()
        self._clear_costmap()
    
        # load model:
        if(model == None):
            model_file = rospy.get_param('~model_file', "./model/drl_vo.zip")
            self.model = PPO.load(model_file, device="cpu")
        else:
            self.model = model
        #print("Finish loading model.")

        # initialize ROS objects
        self.barn_data_sub = rospy.Subscriber("/barn_data", BARN_data, self.barn_data_callback, queue_size=1, buff_size=2**24)
        self.scan_sub = rospy.Subscriber("/front/scan", LaserScan, self.scan_callback)
        self.goal_sub = rospy.Subscriber("/cnn_goal", Point, self.goal_callback)
        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1, latch=False)
        
        self.rate = 20
        self.timer = rospy.Timer(rospy.Duration(1./self.rate), self.timer_callback)

    # Callback function for the scan measurement subscriber
    def scan_callback(self, laserScan_msg):
        # get the laser scan data:
        scan_data = np.array(laserScan_msg.ranges, dtype=np.float32)
        scan_data[np.isnan(scan_data)] = self.LASER_CLIP
        scan_data[np.isinf(scan_data)] = self.LASER_CLIP
        self.scan = scan_data

        # minimum distance:
        scan = np.array(self.scan)  
        scan[scan==0] = self.LASER_CLIP
        self.d_b = np.amin([np.concatenate((scan[:30], scan[-30:]), axis=None)])
        scan_f = scan[240:-240]
        scan_fr = scan_f[:120]
        scan_fl = scan_f[120:]
        self.num_fr = len(scan_fr[scan_fr <= 0.6])
        self.num_fl = len(scan_fl[scan_fl <= 0.6])
        if(scan_f.size!=0):
            self.d_f = np.amin(scan_f)
        else:
            self.d_f = 10

        if(self.d_f <= 0.45):
            if(self.mode == Mode.DRIVE):
                self.mode = Mode.STOP
        else:
            if(self.mode == Mode.TURN):
                self.mode = Mode.STOP


    # Callback function for the current goal subscriber
    def goal_callback(self, goal_msg):
        # Cartesian coordinate:
        self.goal[0] = goal_msg.x
        self.goal[1] = goal_msg.y
        self.theta = abs(np.arctan2(self.goal[1], self.goal[0]))

    # Callback function for the current goal subscriber
    def odom_callback(self, odom_msg):
        # odometry:
        self.odom = odom_msg

    # Callback function for the barn_data subscriber
    def barn_data_callback(self, barn_data_msg):
        # barn data:
        self.barn_data = barn_data_msg
        
        
    # Callback function for the barn_data subscriber
    def timer_callback(self, event):
        self.move_base.make_plan()  
        cmd_vel = Twist()

        if(self.start): # start navigation  
            
            # if the goal is close to the robot:
            if(np.linalg.norm(self.goal) <= 0.9):  # goal margin
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
            else:
                if(self.mode is Mode.STOP):
                    #print("STOP\n")
                    vx = self.odom.twist.twist.linear.x 
                    wz = self.odom.twist.twist.angular.z
                    if(abs(vx) < 0.02 and abs(wz) < 0.02):
                        if(self.d_f <= 0.45):
                            self.mode = Mode.TURN
                        else:
                            self.mode = Mode.DRIVE
                    elif(vx > 0.2):
                        cmd_vel.linear.x = -0.5
                        cmd_vel.angular.z = 0
                    else:
                        cmd_vel.linear.x = 0
                        cmd_vel.angular.z = 0

                elif(self.mode is Mode.DRIVE):
                    #print("DRIVE\n")
                    self.cnt = 0
                    scan_list = self.barn_data.scan
                    goal = self.barn_data.goal_cart

                    cmd_vel = Twist()
                        
                    # ped_map:
                    ped_pos = np.zeros(12800)

                    # scan_map:
                    if(len(scan_list) == 0):
                        laser_tmp = self.scan
                        laser_scan = np.matlib.repmat(laser_tmp,1,10)
                        laser_scan = laser_scan.reshape(7200)
                    else:
                        laser_scan = np.array(scan_list, dtype=np.float32)
                    laser_scan[laser_scan > self.LASER_CLIP] = self.LASER_CLIP
                    # min-avg pooling:
                    scan_avg = np.zeros((20,80))
                    for n in range(10):
                        scan_tmp = laser_scan[n*720:(n+1)*720] - self.jackal_dis + self.turtlebot_dis
                        scan_tmp = scan_tmp[120:-120]
                        for i in range(80):
                            scan_avg[2*n, i] = np.min(scan_tmp[i*6:(i+1)*6])
                            scan_avg[2*n+1, i] = np.mean(scan_tmp[i*6:(i+1)*6])
                    # stacking: 
                    scan_avg = scan_avg.reshape(1600)
                    scan_avg_map = np.matlib.repmat(scan_avg,1,4)
                    scan_map = scan_avg_map.reshape(6400)
                    # MaxAbsScaler:
                    s_min = 0
                    s_max = self.LASER_CLIP
                    scan_map = 2 * (scan_map - s_min) / (s_max - s_min) + (-1)

                    # goal:
                    # MaxAbsScaler:
                    g_min = -1
                    g_max = 1
                    if(len(goal) == 0):
                        goal_orignal = np.array(self.goal, dtype=np.float32)
                    else:
                        goal_orignal = np.array(goal, dtype=np.float32)
                    goal_orignal = np.array(goal, dtype=np.float32)
                    goal = 2 * (goal_orignal - g_min) / (g_max - g_min) + (-1)

                    # observation:
                    self.observation = np.concatenate((ped_pos, scan_map, goal), axis=None) 

                    # drl-vo infrence: calculate the goal velocity of the robot and send the command
                    action, _states = self.model.predict(self.observation, deterministic=True)

                    # velocities:
                    vx_min = 0
                    if(self.d_f >= 2.2): # free space margin
                        vx_max = 2
                        vz_min = -0.7
                        vz_max = 0.7
                    else:
                        vx_max = 0.5 
                        vz_min = -0.7
                        vz_max = 0.7
                    
                    # MaxAbsScaler inverse:
                    cmd_vel.linear.x = (action[0] + 1) * (vx_max - vx_min) / 2 + vx_min
                    cmd_vel.angular.z = (action[1] + 1) * (vz_max - vz_min) / 2 + vz_min
                    

                elif(self.mode is Mode.TURN):
                    #print("TURN\n")
                    if(abs(self.theta) > 1):
                        self.cnt = 0
                    elif(self.d_b <= 0.28 and self.d_f >= 0.25):
                        self.cnt = 6
                        self.move_base.clear_costmap()
                    elif(self.d_f <= 0.3):
                        self.cnt = 0

                    if(self.cnt < 2):
                        self.cnt += 1
                        cmd_vel.linear.x = -0.5
                        cmd_vel.angular.z = 0
                    elif(self.cnt < 4):
                        self.cnt += 1
                        cmd_vel.linear.x = 0
                        cmd_vel.angular.z = 0
                    elif(self.cnt < 8 and self.cnt > 5):
                        self.cnt += 1
                        cmd_vel.linear.x = 0.25
                        cmd_vel.angular.z = 0
                    else:
                        cmd_vel.linear.x = 0
                        if(self.num_fr > self.num_fl - 8):
                            cmd_vel.angular.z = 0.7
                        elif(self.num_fl > self.num_fr - 8):
                            cmd_vel.angular.z = -0.7
                        else:
                            self.cnt = 0
                            cmd_vel.angular.z = 0.7

        
        # publish the cmd_vel:
        if not np.isnan(cmd_vel.linear.x) and not np.isnan(cmd_vel.angular.z): # ensure data is valid
            self.cmd_vel_pub.publish(cmd_vel)
    
    def _clear_costmap(self):
        self.move_base.clear_costmap()
        rospy.sleep(0.1)
        self.move_base.clear_costmap()
        rospy.sleep(0.1)
        self.move_base.clear_costmap()
    #
    # end of function


# begin gracefully
#

if __name__ == '__main__':
    rospy.init_node('drl_vo_inference')
    drl_infe = DrlInference()
    rospy.spin()

# end of file
