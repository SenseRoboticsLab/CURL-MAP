#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
import subprocess
import time
import signal
import os

is_publishing = False
running = True

def terminate_process(process):
    os.killpg(os.getpgid(process.pid), signal.SIGTERM)  # Send SIGTERM signal to the process group
    time.sleep(10)

def signal_handler(signal, frame):
    global running
    print("Stopping the script...")
    running = False


signal.signal(signal.SIGINT, signal_handler)


def check_trajectory_publishing(data):
    global is_publishing
    is_publishing = True

def run_command(sequence):
    # Command for curl_slam odometry_node
    command1 = 'roslaunch aloam_velodyne aloam_velodyne_HDL_64.launch seq:=' + sequence
    process1 = subprocess.Popen(command1, shell=True, preexec_fn=os.setsid)

    # Wait for the specific log message
    # for line in iter(process1.stdout.readline, ''):
    #     print line, # Print command output without newline
    #     if "start update sph coeff thread" in line:
    #         break

    time.sleep(5)  # Wait for 1 second

    # Start kitti launch file
    command2 = 'roslaunch aloam_velodyne kitti_helper.launch seq:=' + sequence
    with open('/dev/null', 'w') as devnull:
        process2 = subprocess.Popen(command2, stdout=devnull, stderr=devnull, shell=True, preexec_fn=os.setsid)
    return process1, process2


def main():
    global is_publishing, running
    rospy.init_node('run_sequence')

    # Subscribe to /curl/trajectory to detect when it stops publishing
    rospy.Subscriber("/laser_odom_path", Path, check_trajectory_publishing)

    for i in range(0,11):  # Iterate through sequences 00 to 10
        sequence = str(i).zfill(2)  # Convert to 00, 01, etc.
        process1, process2 = run_command(sequence)
        time.sleep(10)  # Allow some time for the system to initialize

        # Check if /curl/trajectory is publishing
        while running:
            is_publishing = False
            time.sleep(30)  # Wait for 10 seconds            
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
    # Optionally, you can include code here to kill the running ROS nodes


if __name__ == '__main__':
    main()
