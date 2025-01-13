#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
import subprocess
import time
import signal
import os
import yaml
import numpy as np
from itertools import product

is_publishing = False
running = True


def terminate_process(process):
    # Send SIGTERM signal to the process group
    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    time.sleep(10)


def signal_handler(signal, frame):
    global running
    print("Stopping the script...")
    running = False


signal.signal(signal.SIGINT, signal_handler)


def check_trajectory_publishing(data):
    global is_publishing
    is_publishing = True


def run_command(sequence, config_file):
    # Command for curl_slam odometry_node
    command1 = 'rosrun curl_slam odometry_node ' + config_file + ' ' + sequence
    process1 = subprocess.Popen(command1, shell=True, preexec_fn=os.setsid)

    # Wait for the specific log message
    # while True:
    #     line = temp_file.readline()
    #     print line,  # Print command output without newline
    #     if "start update sph coeff thread" in line:
    #         break

    time.sleep(120)

    # Start kitti launch file
    command2 = 'roslaunch travel run_patchwork_kitti.launch data_dir:=/media/zkc/Seagate\ Portable\ Drive5/DataSets/kitti/data_odometry_velodyne/sequences seq:=' + sequence
    with open('/dev/null', 'w') as devnull:
        process2 = subprocess.Popen(
            command2, stdout=devnull, stderr=devnull, shell=True, preexec_fn=os.setsid)
    # Allow some time for the system to initialize
    time.sleep(10)
    return process1, process2


def update_yaml(extra_skin_thickness, is_save_middle_map, minimum_img_rso, gamma_sampling_interval, middle_map_highest_IoU_new_landmark_thres, base_directory, config_file_path, is_frame_to_frame, weight_of_IoU, largest_tree_key_difference, IQR_scale_factor, max_seg_pairs, max_ground_pairs, highest_IoU_new_landmark_thres, max_new_added_landmarks_num, IQR_max_distance_of_landmarks, is_two_step_opt):
    with open(config_file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)  # It's good practice to use safe_load
        print("finishing load original yaml file")
        # Modifying the parameters with new values
        yaml_data['AABB']['extra_skin_thickness'] = extra_skin_thickness
        yaml_data['AABB']['ground_fatten_scale'] = 0.1/extra_skin_thickness*5
        yaml_data['DEBUG']['MIDDLE_MAP']['is_save_middle_map'] = is_save_middle_map
        yaml_data['DEBUG']['data_set'] = "KITTI"
        yaml_data['DIRECT_METHOD']['CURL_ODOMETRY']['minimum_img_rso'] = minimum_img_rso
        yaml_data['DIRECT_METHOD']['CURL_ODOMETRY']['gamma_sampling_interval'] = gamma_sampling_interval
        yaml_data['DEBUG']['MIDDLE_MAP']['middle_map_highest_IoU_new_landmark_thres'] = middle_map_highest_IoU_new_landmark_thres
        yaml_data['CURL_TRACKING']['is_frame_to_frame'] = is_frame_to_frame
        yaml_data['CURL_TRACKING']['DATA_ASSOCIATION']['weigt_of_IoU'] = float(
            weight_of_IoU)
        if is_save_middle_map:
            yaml_data['CURL_TRACKING']['DATA_ASSOCIATION']['largest_tree_key_difference'] = -1
        else:
            yaml_data['CURL_TRACKING']['DATA_ASSOCIATION']['largest_tree_key_difference'] = largest_tree_key_difference
        yaml_data['CURL_TRACKING']['DATA_ASSOCIATION']['IQR_scale_factor'] = float(
            IQR_scale_factor)
        yaml_data['CURL_TRACKING']['DATA_ASSOCIATION']['max_seg_pairs'] = max_seg_pairs
        yaml_data['CURL_TRACKING']['DATA_ASSOCIATION']['max_ground_pairs'] = max_ground_pairs
        yaml_data['CURL_TRACKING']['NEW_LANDMARK_GENERATION']['highest_IoU_new_landmark_thres'] = float(
            highest_IoU_new_landmark_thres)
        yaml_data['CURL_TRACKING']['NEW_LANDMARK_GENERATION']['max_new_added_landmarks_num'] = max_new_added_landmarks_num
        yaml_data['CURL_TRACKING']['NEW_LANDMARK_GENERATION']['IQR_max_distance_of_landmarks'] = float(
            IQR_max_distance_of_landmarks)
        yaml_data['CURL_TRACKING']['OPTIMIZATION']['is_two_step_opt'] = is_two_step_opt

        # Create the directory if it doesn't exist
        new_directory = None
        if is_frame_to_frame:
            new_directory = base_directory + "/" + "frame_to_frame" + "_" + str(extra_skin_thickness) + "_"+str(minimum_img_rso)+"_"+str(gamma_sampling_interval) + "_" +\
                str(weight_of_IoU) + "_" + str(largest_tree_key_difference) + "_" + \
                str(IQR_scale_factor) + "_" + str(max_seg_pairs) + "_" + str(max_ground_pairs) + "_" + str(
                    highest_IoU_new_landmark_thres) + "_" + str(max_new_added_landmarks_num) + "_" + str(IQR_max_distance_of_landmarks) + "_" + str(is_two_step_opt)
        else:
            new_directory = base_directory + "/" + "frame_to_map" + "_" + str(extra_skin_thickness) + "_" + str(minimum_img_rso)+"_"+str(gamma_sampling_interval) + "_" +\
                str(weight_of_IoU) + "_" + str(largest_tree_key_difference) + "_" + \
                str(IQR_scale_factor) + "_" + str(max_seg_pairs) + "_" + str(max_ground_pairs) + "_" + str(
                    highest_IoU_new_landmark_thres) + "_" + str(max_new_added_landmarks_num) + "_" + str(IQR_max_distance_of_landmarks) + "_" + str(is_two_step_opt)
        if not os.path.exists(new_directory):
            os.makedirs(new_directory)

        yaml_data['DEBUG']['results_dir'] = new_directory
        new_config_file_path = os.path.join(new_directory, "config.yaml")

        # Writing the new YAML data
        with open(new_config_file_path, 'w') as new_file:
            yaml.dump(yaml_data, new_file, default_flow_style=False)

        return new_config_file_path, new_directory


def main():
    global is_publishing, running
    rospy.init_node('run_sequence')
    base_directory = '/media/zkc/EAGET/icra_24/curl_odom_diff_param_evaluation'
    original_config_file_path = '/home/user/project/ros/slam/src/CURL_SLAM_ros/config/config.yaml'
    # Subscribe to /curl/trajectory to detect when it stops publishing
    rospy.Subscriber("/curl/trajectory", Path, check_trajectory_publishing)

    for extra_skin_thickness, is_save_middle_map, minimum_img_rso, gamma_sampling_interval, middle_map_highest_IoU_new_landmark_thres, is_frame_to_frame, weight_of_IoU, largest_tree_key_difference, IQR_scale_factor, max_seg_pairs, max_ground_pairs, highest_IoU_new_landmark_thres, max_new_added_landmarks_num, IQR_max_distance_of_landmarks, is_two_step_opt in product(
        [0.1],  # extra_skin_thickness
        [False],  # is_save_middle_map
        [15],  # minimum_img_rso
        [0.05],  # gamma_sampling_interval
        [0.15],  # middle_map_highest_IoU_new_landmark_thres
        [False],  # is_frame_to_frame
        # np.arange(1, 0.5, -0.1),  # weight_of_IoU
        [1],  # weight_of_IoU
        [10000, -1],  # largest_tree_key_difference
        [0.5],  # IQR_scale_factor
        [-1],  # max_seg_pairs
        [-1],  # max_ground_pairs
        # highest_IoU_new_landmark_thres np.arange(0.05, 0.1, 0.01)
        [0.05],
        [-1],  # max_new_added_landmarks_num
        # IQR_max_distance_of_landmarks
        # np.append(np.arange(0, 0.75, 0.25), -100),
        [-100],
        [True]  # is_two_step_opt [True, False]
    ):

        new_config_file_path, new_directory = update_yaml(extra_skin_thickness, is_save_middle_map, minimum_img_rso, gamma_sampling_interval, middle_map_highest_IoU_new_landmark_thres, base_directory,
                                                          original_config_file_path, is_frame_to_frame, weight_of_IoU, largest_tree_key_difference, IQR_scale_factor, max_seg_pairs, max_ground_pairs, highest_IoU_new_landmark_thres, max_new_added_landmarks_num, IQR_max_distance_of_landmarks, is_two_step_opt)
        print("************************")
        print(new_directory)
        # Iterate through sequences 00 to 10
        for i in range(0, 11):
            if os.path.exists(os.path.join(new_directory, "KITTI_trajectory_"+str(i).zfill(2))):
                print(str(i).zfill(2)+" exist")
                continue
            else:
                print("process: " + str(i).zfill(2))
            # Convert to 00, 01, etc.
            sequence = str(i).zfill(2)
            process1, process2 = run_command(
                sequence, new_config_file_path)
            # Check if /curl/trajectory is publishing
            while running:
                is_publishing = False
                # Wait for 10 seconds
                time.sleep(10)
                if not is_publishing:
                    print("finished publishing")
                    terminate_process(process1)
                    terminate_process(process2)
                    break  # Break loop if no publishing detected in 10 seconds
            if not running:
                print("stop")
                terminate_process(process1)
                terminate_process(process2)
                break
        print("Exiting...")

        if not running:
            break
# Optionally, you can include code here to kill the running ROS nodes


if __name__ == '__main__':
    main()
