import os, sys
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_pdf import PdfPages

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math
import copy

from evo.core import metrics
from evo.tools.settings import SETTINGS
from evo.tools import plot
from evo.core import sync
from evo.core.trajectory import PoseTrajectory3D

import rosbag
import transformations

def evo_stuff(file):
    topic = '/kdvisual_ros/pose'

    proj_2d = False

    kd_timestamp = np.array([])
    kd_x = np.array([])
    kd_y = np.array([])
    kd_z = np.array([])
    kd_roll = np.array([])
    kd_pitch = np.array([])
    kd_yaw = np.array([])
    kd_q = np.array([])

    bag = rosbag.Bag(file)
    for topic, msg, t in bag.read_messages(topics=[topic]):

        kd_timestamp = np.append(kd_timestamp, msg.header.stamp.to_sec())

        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = transformations.euler_from_quaternion (orientation_list)

        kd_x = np.append(kd_x, msg.pose.position.x)
        kd_y = np.append(kd_y, msg.pose.position.y)
        if not proj_2d:
            kd_z = np.append(kd_z, msg.pose.position.z)
        else:
            kd_z = np.append(kd_z, 0)

        kd_yaw = np.append(kd_yaw, yaw)
        kd_pitch = np.append(kd_pitch, pitch)
        kd_roll = np.append(kd_roll, roll)

        if not proj_2d:
            q = np.array(orientation_list)
        else:
            q = transformations.quaternion_from_euler (0, 0, yaw)

        if len(kd_q):
            kd_q = np.vstack((kd_q, q))
        else: 
            kd_q = np.append(kd_q, q)

 #   bag.close()

    print ('Processed', len(kd_timestamp), 'messages')

    kd_timestamp = kd_timestamp - kd_timestamp[0]
    kd_xyz = np.column_stack((kd_x, kd_y, kd_z))
    traj_kd = PoseTrajectory3D(kd_xyz, kd_q, kd_timestamp)
    print (traj_kd)

    topic = '/vrpn_client_node/Gemini0/pose'

    proj_2d = False

    gt_timestamp = np.array([])
    gt_x = np.array([])
    gt_y = np.array([])
    gt_z = np.array([])
    gt_roll = np.array([])
    gt_pitch = np.array([])
    gt_yaw = np.array([])
    gt_q = np.array([])

    for topic, msg, t in bag.read_messages(topics=[topic]):

        gt_timestamp = np.append(gt_timestamp, msg.header.stamp.to_sec())

        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = transformations.euler_from_quaternion (orientation_list)

        gt_x = np.append(gt_x, msg.pose.position.x)
        gt_y = np.append(gt_y, msg.pose.position.y)
        if not proj_2d:
            gt_z = np.append(gt_z, msg.pose.position.z)
        else:
            gt_z = np.append(gt_z, 0)

        gt_yaw = np.append(gt_yaw, yaw)
        gt_pitch = np.append(gt_pitch, pitch)
        gt_roll = np.append(gt_roll, roll)

        if not proj_2d:
            q = np.array(orientation_list)
        else:
            q = transformations.quaternion_from_euler (0, 0, yaw)

        if len(gt_q):
            gt_q = np.vstack((gt_q, q))
        else: 
            gt_q = np.append(gt_q, q)

    bag.close()

    print ('Processed', len(gt_timestamp), 'messages')

    gt_timestamp = gt_timestamp - gt_timestamp[0]
    gt_xyz = np.column_stack((gt_x, gt_y, gt_z))
    traj_gt = PoseTrajectory3D(gt_xyz, gt_q, gt_timestamp)
    print (traj_gt)

    plotxyz (traj_kd, traj_gt)

def plotxyz(traj_kd, traj_gt):
    
    # Plot change in x y z axes (KdVisual = black & GT = blue)

    max_diff = 0.01
    #traj_kd_aligned = copy.deepcopy(traj_kd)
    #traj_kd_aligned.align(traj_gt, correct_scale=False, correct_only_scale=False)
    traj_gt, traj_kd = sync.associate_trajectories(traj_gt, traj_kd, max_diff)


    fig, axarr = plt.subplots(3)
    plot.traj_xyz(axarr, traj_kd)
    plot.traj_xyz(axarr, traj_gt, color='blue')
    plt.show(block=False)

    plt.pause(0.1)


if __name__ == "__main__":
   
    current_directory = os.getcwd()

    # List all entries in the current working directory and filter out directories
    entries = os.listdir(current_directory)
    bag_files = []
    for root, dirs, files in os.walk(current_directory):
        for file in files:
            if file.endswith('.bag'):
                file = os.path.join(root, file)
                print(file)
                evo_stuff(file)
   # file = '\n'.join(bag_files)
   # evo_stuff(bag_files)
   # print(file)
 #               print(f"Checking directory: {logs_file_path}")
 #               print(f"Checking directory: {traj_file_path}")
                # Process the 'logs' file 

             #   print(f"Trajectory file: {traj_file_path}")
             #   exceedance_percentage = check_processing_time(logs_file_path, fps)
             #   lost_percentage, all_lost_in_first_two_seconds = calculate_lost_percentage(logs_file_path)
             #   all_lost_in_first_five_seconds, all_lost_in_first_ten_seconds = cold_start_home_time(logs_file_path)
             #   vertical_exceedance_percentage = calculate_exceedance_percentage(traj_file_path)
                # Save the result in the dictionary with the file path as the key
             #   exceedance_percentages[dir_name] = exceedance_percentage
#                else:
#                    print(f"File not found: {logs_file_path}")

#    print(f"\n<----- Test: {dir_name} ----->")
#    print(f"Log file: {file}")
