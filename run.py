import time
import argparse
import subprocess
import os
from os.path import join

import numpy as np
import rospy
import rospkg
from roslauncher import ROSLauncher

from gazebo_simulation import GazeboSimulation

INIT_POSITION = [-2, 3, 1.57]  # in world frame
GOAL_POSITION = [0, 10]  # relative to the initial position

def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def path_coord_to_gazebo_coord(x, y):
        RADIUS = 0.075
        r_shift = -RADIUS - (30 * RADIUS * 2)
        c_shift = RADIUS + 5

        gazebo_x = x * (RADIUS * 2) + r_shift
        gazebo_y = y * (RADIUS * 2) + c_shift

        return (gazebo_x, gazebo_y)

class RUN_DRL_VO:
    def __init__(self, world_idx=0, gui=True, out_path="out.txt"):
        # parameters:
        self.gui = gui
        self.out_path = out_path

        ##########################################################################################
        ## 0. Launch Gazebo Simulation
        ##########################################################################################
        rospack = rospkg.RosPack()
        self.BASE_PATH = rospack.get_path('jackal_helper')
        world = "BARN/world_%d.world" %(world_idx)
        world_name = join(self.BASE_PATH, "worlds", world)
        world_argument = 'world_name:=' + world_name
        gui_argument = 'gui:=' + str(self.gui)
        gazebo_arguments = [world_argument, gui_argument]
        print(">>>>>>>>>>>>>>>>>> Loading Gazebo Simulation with %s <<<<<<<<<<<<<<<<<<" %(world))   
        self.gazebo_launcher = ROSLauncher(rospackage_name="jackal_helper",\
                                           launch_file_name="gazebo_launch.launch",\
                                           arguments=gazebo_arguments)    
        time.sleep(6)  # sleep to wait until the gazebo being created
        
        # GazeboSimulation provides useful interface to communicate with gazebo  
        self.gazebo_sim = GazeboSimulation(init_position=INIT_POSITION)
        
        self.init_coor = (INIT_POSITION[0], INIT_POSITION[1])
        self.goal_coor = (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1])
        
        pos = self.gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        collided = True
        
        # check whether the robot is reset, the collision is False
        while compute_distance(self.init_coor, curr_coor) > 0.1 or collided:
            self.gazebo_sim.reset() # Reset to the initial position
            pos = self.gazebo_sim.get_model_state().pose.position
            curr_coor = (pos.x, pos.y)
            collided = self.gazebo_sim.get_hard_collision()
            time.sleep(1)

        ##########################################################################################
        ## 1. Launch your navigation stack
        ## (Customize this block to add your own navigation stack)
        ########################################################################################## 
        start = False
        start_argument = 'start:=' + str(start)
        goal_x_argument = 'goal_x:=' + str(GOAL_POSITION[0])
        goal_y_argument = 'goal_y:=' + str(GOAL_POSITION[1])
        nav_arguments = [start_argument, goal_x_argument, goal_y_argument] 
        self.navigation_launcher = ROSLauncher(rospackage_name="jackal_helper",\
                                               launch_file_name="move_base_drl_vo.launch",\
                                               arguments=nav_arguments)    
        time.sleep(2)

    def start_naviagtion(self, world_idx=0):
        ##########################################################################################
        ## 0. Launch Gazebo Simulation
        ##########################################################################################
        world = "BARN/world_%d.world" %(world_idx)
        world_name = join(self.BASE_PATH, "worlds", world)
        world_argument = 'world_name:=' + world_name
        gui_argument = 'gui:=' + str(self.gui)
        gazebo_arguments = [world_argument, gui_argument]
        print(">>>>>>>>>>>>>>>>>> Loading Gazebo Simulation with %s <<<<<<<<<<<<<<<<<<" %(world))   
        self.gazebo_launcher.restart(arguments=gazebo_arguments)    
        time.sleep(6)  # sleep to wait until the gazebo being created

        # reset gazebo state: 
        self.gazebo_sim.reset()
        
        pos = self.gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        collided = True
        
        # check whether the robot is reset, the collision is False
        while compute_distance(self.init_coor, curr_coor) > 0.1 or collided:
            self.gazebo_sim.reset() # Reset to the initial position
            pos = self.gazebo_sim.get_model_state().pose.position
            curr_coor = (pos.x, pos.y)
            collided = self.gazebo_sim.get_hard_collision()
            time.sleep(1)

        ##########################################################################################
        ## 1. Launch your navigation stack
        ## (Customize this block to add your own navigation stack)
        ########################################################################################## 
        start = True
        start_argument = 'start:=' + str(start)
        goal_x_argument = 'goal_x:=' + str(GOAL_POSITION[0])
        goal_y_argument = 'goal_y:=' + str(GOAL_POSITION[1])
        nav_arguments = [start_argument, goal_x_argument, goal_y_argument] 
        self.navigation_launcher.restart(arguments=nav_arguments)
        time.sleep(2)

        ##########################################################################################
        ## 2. Start navigation
        ##########################################################################################
        
        curr_time = rospy.get_time()
        pos = self.gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        
        # check whether the robot started to move
        while compute_distance(self.init_coor, curr_coor) < 0.1:
            curr_time = rospy.get_time()
            pos = self.gazebo_sim.get_model_state().pose.position
            curr_coor = (pos.x, pos.y)
            time.sleep(0.01)
        
        # start navigation, check position, time and collision
        start_time = curr_time
        start_time_cpu = time.time()
        collided = False
        
        while compute_distance(self.goal_coor, curr_coor) > 1 and not collided and curr_time - start_time < 100:
            curr_time = rospy.get_time()
            pos = self.gazebo_sim.get_model_state().pose.position
            curr_coor = (pos.x, pos.y)
            print("Time: %.2f (s), x: %.2f (m), y: %.2f (m)" %(curr_time - start_time, *curr_coor), end="\r")
            collided = self.gazebo_sim.get_hard_collision()
            while rospy.get_time() - curr_time < 0.1:
                time.sleep(0.01)

        
        ##########################################################################################
        ## 3. Report metrics and generate log
        ##########################################################################################
        
        print(">>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<")
        success = False
        if collided:
            status = "collided"
        elif curr_time - start_time >= 100:
            status = "timeout"
        else:
            status = "succeeded"
            success = True
        print("Navigation %s with time %.4f (s)" %(status, curr_time - start_time))
        
        path_file_name = join(self.BASE_PATH, "worlds/BARN/path_files", "path_%d.npy" %world_idx)
        path_array = np.load(path_file_name)
        path_array = [path_coord_to_gazebo_coord(*p) for p in path_array]
        path_array = np.insert(path_array, 0, (INIT_POSITION[0], INIT_POSITION[1]), axis=0)
        path_array = np.insert(path_array, len(path_array), (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1]), axis=0)
        path_length = 0
        for p1, p2 in zip(path_array[:-1], path_array[1:]):
            path_length += compute_distance(p1, p2)
        
        # Navigation metric: 1_success *  optimal_time / clip(actual_time, 4 * optimal_time, 8 * optimal_time)
        optimal_time = path_length / 2
        actual_time = curr_time - start_time
        nav_metric = int(success) * optimal_time / np.clip(actual_time, 4 * optimal_time, 8 * optimal_time)
        print("Navigation metric: %.4f" %(nav_metric))
        
        with open(self.out_path, "a") as f:
            f.write("%d %d %d %d %.4f %.4f\n" %(world_idx, success, collided, (curr_time - start_time)>=100, curr_time - start_time, nav_metric))
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = 'test BARN navigation challenge')
    parser.add_argument('--world_idx', type=int, default=0)
    parser.add_argument('--gui', action="store_true")
    parser.add_argument('--out', type=str, default="out.txt")
    args = parser.parse_args()

    # environment parameters:
    os.environ["JACKAL_LASER"] = "1"
    os.environ["JACKAL_LASER_MODEL"] = "ust10"
    os.environ["JACKAL_LASER_OFFSET"] = "-0.065 0 0.01"

    # launch roscore:
    roscore_process = subprocess.Popen('roscore')
    time.sleep(1)

    # create node:
    rospy.init_node('gym', anonymous=True) #, log_level=rospy.FATAL)
    rospy.set_param('/use_sim_time', True)

    # initialize running test:
    run_drl_vo = RUN_DRL_VO(world_idx=args.world_idx, gui=args.gui, out_path=args.out)

    # run test:
    run_drl_vo.start_naviagtion(args.world_idx)
    
    # close roscore:
    roscore_process.terminate()
    
    
