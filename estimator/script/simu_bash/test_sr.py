# ! /usr/bin/python2
# Usage: python run_sr_test.py -sequence=SR -program=mc_test -start_idx=0 -end_idx=4 -trials=10

import os
import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

seq_name = []

def debug_test(start_idx, end_idx):
    global seq_name
    for idx in range(start_idx, end_idx + 1):
        print('testing sequence: {}'.format(seq_name[idx]))
        os.environ['data_path'] = '{}/lidar_calibration/mloam_dataset/{}.bag'.format(os.environ['DATA_PATH'], seq_name[idx])
        os.environ['rpg_path'] = '{}/src/localization/rpg_trajectory_evaluation'.format(os.environ['CATKIN_WS'])
        os.environ['result_path'] = '{}/results/simu_jackal_mloam/debug/'.format(os.environ['rpg_path'])
        command = 'mkdir -p $result_path/gf_pcd'
        os.system(command)
        command = 'mkdir -p $result_path/traj'
        os.system(command)
        command = 'mkdir -p $result_path/time'
        os.system(command)
        command = 'mkdir -p $result_path/pose_graph'
        os.system(command)
        command = 'mkdir -p $result_path/others'
        os.system(command)
        command = 'bash test_main.sh'
        os.system(command)

def single_test(start_idx, end_idx):
    global seq_name
    for idx in range(start_idx, end_idx + 1):
        print('testing sequence: {}'.format(seq_name[idx]))
        os.environ['data_path'] = '{}/lidar_calibration/mloam_dataset/{}.bag'.format(os.environ['DATA_PATH'], seq_name[idx])
        os.environ['rpg_path'] = '{}/src/localization/rpg_trajectory_evaluation'.format(os.environ['CATKIN_WS'])
        os.environ['result_path'] = '{}/results/simu_jackal_mloam/{}/'.format(os.environ['rpg_path'], seq_name[idx])
        command = 'mkdir -p $result_path/gf_pcd'
        os.system(command)
        command = 'mkdir -p $result_path/traj'
        os.system(command)
        command = 'mkdir -p $result_path/time'
        os.system(command)
        command = 'mkdir -p $result_path/pose_graph'
        os.system(command)
        command = 'mkdir -p $result_path/others'
        os.system(command)
        command = 'bash test_main.sh'
        os.system(command)

        print('evaluate sequence: {}'.format(seq_name[idx]))
        command = 'python2 $rpg_path/scripts/analyze_trajectory_single_mloam.py '
        command += '--recalculate_errors --est_type M-LO M-LOAM-wo-ua M-LOAM A-LOAM F-LOAM --compare $result_path/traj'
        os.system(command)          

def single_eval(start_idx, end_idx):
    global seq_name
    for idx in range(start_idx, end_idx + 1):
        print('evaluate sequence: {}'.format(seq_name[idx]))
        os.environ['rpg_path'] = '{}/src/localization/rpg_trajectory_evaluation'.format(os.environ['CATKIN_WS'])
        os.environ['result_path'] = '{}/results/simu_jackal_mloam/{}/'.format(os.environ['rpg_path'], seq_name[idx])
        command = 'python2 $rpg_path/scripts/analyze_trajectory_single_mloam.py '
        command += '--recalculate_errors --est_type M-LO M-LOAM-wo-ua M-LOAM A-LOAM F-LOAM --compare $result_path/traj'
        os.system(command)            

def mc_test(start_idx, end_idx, mc_trials):
    global seq_name
    for idx in range(start_idx, end_idx + 1):
        print('testing sequence: {}'.format(seq_name[idx]))
        os.environ['rpg_path'] = '{}/src/localization/rpg_trajectory_evaluation'.format(os.environ['CATKIN_WS'])
        os.environ['result_path'] = '{}/results/simu_jackal_mloam/monte_carlo_{}/'.format(os.environ['rpg_path'], seq_name[idx])
        command = 'mkdir -p $result_path/gf_pcd'
        os.system(command)
        command = 'mkdir -p $result_path/traj'
        os.system(command)
        command = 'mkdir -p $result_path/time'
        os.system(command)
        command = 'mkdir -p $result_path/pose_graph'
        os.system(command)
        command = 'mkdir -p $result_path/others'
        os.system(command)
    
        for trial in range(0, mc_trials):
            print('mc_trial {}'.format(trial))
            os.environ['data_path'] = '{}/lidar_calibration/mloam_dataset/SR_monte_carlo/group_{}/{}'.format(os.environ['DATA_PATH'], trial, idx)
            command = 'bash test_main_monte_carlo.sh'
            os.system(command)
            command = 'mv $result_path/traj/stamped_groundtruth.txt $result_path/traj/stamped_groundtruth{}.txt'.format(trial)
            os.system(command)
            command = 'mv $result_path/traj/stamped_mloam_odom_estimate_1.000000.txt $result_path/traj/stamped_mloam_odom_estimate_1.000000{}.txt'.format(trial)
            os.system(command)
            command = 'mv $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_0{}.txt'.format(trial)
            os.system(command)
            command = 'mv $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_0{}.txt'.format(trial)
            os.system(command)
            command = 'mv $result_path/traj/stamped_aloam_odom_estimate.txt $result_path/traj/stamped_aloam_odom_estimate{}.txt'.format(trial)
            os.system(command)
            command = 'mv $result_path/traj/stamped_aloam_map_estimate.txt $result_path/traj/stamped_aloam_map_estimate{}.txt'.format(trial)
            os.system(command)
            command = 'mv $result_path/traj/stamped_floam_map_estimate.txt $result_path/traj/stamped_floam_map_estimate{}.txt'.format(trial)
            os.system(command)

        command = 'cp $result_path/traj/stamped_groundtruth{}.txt $result_path/traj/stamped_groundtruth.txt'.format(0)
        os.system(command)
        # command = 'python2 $rpg_path/scripts/analyze_trajectory_single_mloam.py '
        # command += '--recalculate_errors --est_type M-LO M-LOAM-wo-ua M-LOAM A-LOAM F-LOAM --compare $result_path/traj '
        # command += '--mul_trials={}'.format(mc_trials)
        # os.system(command)

def mc_eval(start_idx, end_idx, mc_trials):
    global seq_name
    for idx in range(start_idx, end_idx + 1):
        print('evaluate sequence: {}'.format(seq_name[idx]))
        os.environ['rpg_path'] = '{}/src/localization/rpg_trajectory_evaluation'.format(os.environ['CATKIN_WS'])
        os.environ['result_path'] = '{}/results/simu_jackal_mloam/monte_carlo_{}/'.format(os.environ['rpg_path'], seq_name[idx])
        command = 'python2 $rpg_path/scripts/analyze_trajectory_single_mloam.py '
        command += '--recalculate_errors --est_type M-LO M-LOAM-wo-ua M-LOAM A-LOAM F-LOAM --compare $result_path/traj '
        command += '--mul_trials={}'.format(mc_trials)
        os.system(command)    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'run mloam sr test')
    parser.add_argument('-program', help = 'program')
    parser.add_argument('-sequence', help = 'sequence')
    parser.add_argument('-start_idx', type=int, help = 'start sequence')
    parser.add_argument('-end_idx', type=int, help = 'end sequence')
    parser.add_argument('-mc_trials', type=int, help = 'mc trials')
    args = parser.parse_args()
    global seq_name
    if args.sequence == 'SR':
        seq_name = ['SR01', 'SR02', 'SR03', 'SR04', 'SR05']
    elif args.sequence == 'RHD':
        seq_name = ['RHD02lab', 'RHD03garden', 'RHD04building']
    if args.program == 'single_test':
        single_test(args.start_idx, args.end_idx)
    elif args.program == 'debug_test':
        debug_test(args.start_idx, args.end_idx)
    elif args.program =='single_eval':
        single_eval(args.start_idx, args.end_idx)        
    elif args.program == 'mc_test':
        mc_test(args.start_idx, args.end_idx, args.mc_trials)
    elif args.program =='mc_eval':
        mc_eval(args.start_idx, args.end_idx, args.mc_trials)

