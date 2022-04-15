from collections import defaultdict
from typing import NamedTuple
import argparse
from os.path import join
import rospy
import rospkg

import numpy as np

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

class NavLog(NamedTuple):
    world_idx: int
    succeeded: bool
    collided: bool
    timeout: bool
    time: float
    nav_metric: float
        
if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description = 'report test results')
    parser.add_argument("--out_path", type=str, help="path to the out file generated by the test")
    
    args = parser.parse_args()
    
    rospack = rospkg.RosPack()
    base_path = rospack.get_path('jackal_helper')
    optimal_times = {}
    cell_counts = {}
    for idx in range(50):
        path_file_name = join(base_path, "worlds/BARN/path_files", "path_%d.npy" %(idx * 6))
        path_array = np.load(path_file_name)
        path_array = [path_coord_to_gazebo_coord(*p) for p in path_array]
        path_array = np.insert(path_array, 0, (INIT_POSITION[0], INIT_POSITION[1]), axis=0)
        path_array = np.insert(path_array, len(path_array), (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1]), axis=0)
        path_length = 0
        for p1, p2 in zip(path_array[:-1], path_array[1:]):
            path_length += compute_distance(p1, p2)
        optimal_times[idx * 6] = path_length / 2

    results = defaultdict(list)
    with open(args.out_path, "r") as f:
        for l in f.readlines():
            logs = l.split(" ")
            world_idx = int(logs[0])
            nav_log = NavLog(
                world_idx,
                bool(int(logs[1])),
                bool(int(logs[2])),
                bool(int(logs[3])),
                float(logs[4]),
                int(logs[1]) * optimal_times[world_idx] / np.clip(float(logs[4]), optimal_times[world_idx] * 4, optimal_times[world_idx] * 8)  # 1_success * optimal_time / clip(actual_time, 2 * optimal_time, 4 * optimal_time)
            )
            results[world_idx].append(nav_log)

    for idx in range(50):
        if not idx * 6 in results.keys():
            print("Missing world_%d" %(idx * 6))
        elif len(results[idx * 6]) < 10:
            print("Test on world_%d not finished (%d/10)" %(idx * 6, len(results[idx * 6])))

    mean_time = []
    for k in results.keys():
        mean_time_world = [nl.time for nl in results[k] if nl.succeeded]
        if len(mean_time_world) > 0:
            mean_time.append(np.mean(mean_time_world))
    print("Avg Time: %.4f, Avg Metric: %.4f, Avg Success: %.4f, Avg Collision: %.4f, Avg Timeout: %.4f" %(
        np.mean(mean_time),
        np.mean([np.mean([nl.nav_metric for nl in results[k]]) for k in results.keys()]),
        np.mean([np.mean([nl.succeeded for nl in results[k]]) for k in results.keys()]),
        np.mean([np.mean([nl.collided for nl in results[k]]) for k in results.keys()]),
        np.mean([np.mean([nl.timeout for nl in results[k]]) for k in results.keys()]),
    ))
