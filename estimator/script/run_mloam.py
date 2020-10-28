# ! /usr/bin/python2
# Usage: python run_sr_test.py -sequence=SR -program=mc_test -start_idx=0 -end_idx=4 -trials=10

import os
import sys
import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

seq_name = []
platform = ''
seq_main_name = ''
est_type = ''
yaml_name = ''

# python2 run_mloam.py -sequence=SR -program=calib_test -start_idx=0 
def calib_test(start_idx, end_idx):
    for idx in range(start_idx, end_idx + 1):
        print('testing sequence: {}'.format(seq_name[idx]))
        os.environ['data_path'] = '{}/lidar_calibration/mloam_dataset/{}.bag'.format(os.environ['DATA_PATH'], seq_name[idx])
        os.environ['rpg_path'] = '{}/src/localization/rpg_trajectory_evaluation'.format(os.environ['CATKIN_WS'])
        os.environ['result_path'] = '{}/results/{}/calib_{}/'.format(os.environ['rpg_path'], platform, seq_name[idx])
        command = 'mkdir -p $result_path/gf_pcd $result_path/traj $result_path/time \
                            $result_path/pose_graph $result_path/others $result_path/gf_pcd'
        os.system(command)
        command = 'bash {}'.format(seq_main_name)
        os.system(command)
        command = 'cp ~/catkin_ws/src/localization/M-LOAM/estimator/config/{} {}'.format(yaml_name, os.environ['result_path'])
        os.system(command)

# python2 run_mloam.py -sequence=SR -program=debug_test -start_idx=0 
def debug_test(start_idx):
    idx = start_idx
    print('testing sequence: {}'.format(seq_name[idx]))
    # os.environ['data_path'] = '{}/lidar_calibration/mloam_dataset/SR_monte_carlo/group_1/{}.bag'.format(os.environ['DATA_PATH'], seq_name[idx])
    os.environ['data_path'] = '{}/lidar_calibration/mloam_dataset/{}.bag'.format(os.environ['DATA_PATH'], seq_name[idx])
    os.environ['rpg_path'] = '{}/src/localization/rpg_trajectory_evaluation'.format(os.environ['CATKIN_WS'])
    os.environ['result_path'] = '{}/results/{}/debug/'.format(os.environ['rpg_path'], platform)
    command = 'mkdir -p $result_path/gf_pcd $result_path/traj $result_path/time \
                        $result_path/pose_graph $result_path/others $result_path/gf_pcd'
    os.system(command)
    command = 'bash {}'.format(seq_main_name)
    os.system(command)
    command = 'cp ~/catkin_ws/src/localization/M-LOAM/estimator/config/{} {}'.format(yaml_name, os.environ['result_path'])
    os.system(command)    

# python2 run_mloam.py -sequence=SR -program=debug_eval 
def debug_eval():
    print('evaluate debug sequence')
    os.environ['rpg_path'] = '{}/src/localization/rpg_trajectory_evaluation'.format(os.environ['CATKIN_WS'])
    os.environ['result_path'] = '{}/results/{}/debug/'.format(os.environ['rpg_path'], platform)
    command = 'python2 $rpg_path/scripts/analyze_trajectory_single_mloam.py \
               --recalculate_errors --est_type {} \
               --compare $result_path/traj'.format(est_type)
    os.system(command)         

# python2 run_mloam.py -sequence=SR -program=single_test \
#   -start_idx=0 -end_idx=4
def single_test(start_idx, end_idx):
    for idx in range(start_idx, end_idx + 1):
        print('testing sequence: {}'.format(seq_name[idx]))
        # os.environ['data_path'] = '{}/lidar_calibration/mloam_dataset/SR_monte_carlo/group_1/{}.bag'.format(os.environ['DATA_PATH'], seq_name[idx])
        os.environ['data_path'] = '{}/lidar_calibration/mloam_dataset/{}.bag'.format(os.environ['DATA_PATH'], seq_name[idx])
        os.environ['rpg_path'] = '{}/src/localization/rpg_trajectory_evaluation'.format(os.environ['CATKIN_WS'])
        os.environ['result_path'] = '{}/results/{}/{}/'.format(os.environ['rpg_path'], platform, seq_name[idx])
        command = 'mkdir -p $result_path/gf_pcd $result_path/traj $result_path/time \
                            $result_path/pose_graph $result_path/others $result_path/gf_pcd'
        os.system(command)
        command = 'bash {}'.format(seq_main_name)
        os.system(command)
        command = 'cp ~/catkin_ws/src/localization/M-LOAM/estimator/config/{} {}'.format(yaml_name, os.environ['result_path'])
        os.system(command)               
        command = 'cp /tmp/mloam_mapping_surf_cloud.pcd {}'.format(os.environ['result_path'])
        os.system(command)
        command = 'cp /tmp/mloam_mapping_surf_cloud_wo_ua.pcd {}'.format(os.environ['result_path'])
        os.system(command)
        command = 'cp /tmp/aloam_mapping.pcd {}'.format(os.environ['result_path'])
        os.system(command)        

# python2 run_mloam.py -sequence=SR -program=single_eval \
#   -start_idx=0 -end_idx=4
def single_eval(start_idx, end_idx):
    for idx in range(start_idx, end_idx + 1):
        print('evaluate sequence: {}'.format(seq_name[idx]))
        os.environ['rpg_path'] = '{}/src/localization/rpg_trajectory_evaluation'.format(os.environ['CATKIN_WS'])
        os.environ['result_path'] = '{}/results/{}/{}/'.format(os.environ['rpg_path'], platform, seq_name[idx])
        command = 'python2 $rpg_path/scripts/analyze_trajectory_single_mloam.py \
                    --recalculate_errors --est_type {} \
                    --compare $result_path/traj'.format(est_type)
        os.system(command)            

# python2 run_mloam.py -sequence=SR -program=mc_test \
#   -start_idx=0 -end_idx=4 -mc_trials=10
def mc_test(start_idx, end_idx, mc_trials):
    for idx in range(start_idx, end_idx + 1):
        print('testing sequence: {}'.format(seq_name[idx]))
        os.environ['rpg_path'] = '{}/src/localization/rpg_trajectory_evaluation'.format(os.environ['CATKIN_WS'])
        os.environ['result_path'] = '{}/results/{}/monte_carlo_{}/'.format(os.environ['rpg_path'], platform, seq_name[idx])
        print(os.environ['result_path'])
        command = 'mkdir -p $result_path/gf_pcd $result_path/traj $result_path/time \
                            $result_path/pose_graph $result_path/others $result_path/gf_pcd'
        os.system(command)
        for trial in range(0, mc_trials):
            print('mc_trial {}'.format(trial))
            # os.environ['data_path'] = '{}/lidar_calibration/mloam_dataset/SR_monte_carlo/group_{}/{}.bag'.format(os.environ['DATA_PATH'], trial, seq_name[idx])
            os.environ['data_path'] = '{}/lidar_calibration/mloam_dataset/{}.bag'.format(os.environ['DATA_PATH'], seq_name[idx])
            command = 'bash {}'.format(seq_main_name)
            os.system(command)        
            command = 'mv $result_path/traj/stamped_groundtruth.txt $result_path/traj/stamped_groundtruth{}.txt'.format(trial)
            os.system(command)
            command = 'mv $result_path/traj/stamped_mloam_odom_estimate_1.000000.txt $result_path/traj/stamped_mloam_odom_estimate_1.000000{}.txt'.format(trial)
            os.system(command)
            command = 'mv $result_path/traj/stamped_mloam_odom_estimate_0.800000.txt $result_path/traj/stamped_mloam_odom_estimate_0.800000{}.txt'.format(trial)
            os.system(command)

            command = 'mv $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000.txt $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000{}.txt'.format(trial)
            os.system(command)
            command = 'mv $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000.txt $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000{}.txt'.format(trial)
            os.system(command)
            command = 'mv $result_path/traj/stamped_mloam_map_estimate_gd_float_0.200000.txt $result_path/traj/stamped_mloam_map_estimate_gd_float_0.200000{}.txt'.format(trial)
            os.system(command)
            command = 'mv $result_path/traj/stamped_mloam_map_estimate_rnd_0.200000.txt $result_path/traj/stamped_mloam_map_estimate_rnd_0.200000{}.txt'.format(trial)
            os.system(command)            
            command = 'mv $result_path/traj/stamped_mloam_map_estimate_fps_0.200000.txt $result_path/traj/stamped_mloam_map_estimate_fps_0.200000{}.txt'.format(trial)
            os.system(command)                        
            command = 'mv $result_path/traj/stamped_mloam_map_estimate_gd_fix_0.200000.txt $result_path/traj/stamped_mloam_map_estimate_gd_fix_0.200000{}.txt'.format(trial)
            os.system(command)                        

            command = 'mv $result_path/traj/stamped_aloam_odom_estimate.txt $result_path/traj/stamped_aloam_odom_estimate{}.txt'.format(trial)
            os.system(command)
            command = 'mv $result_path/traj/stamped_aloam_map_estimate.txt $result_path/traj/stamped_aloam_map_estimate{}.txt'.format(trial)
            os.system(command)
            command = 'mv $result_path/traj/stamped_floam_map_estimate.txt $result_path/traj/stamped_floam_map_estimate{}.txt'.format(trial)
            os.system(command)
            command = 'mv $result_path/traj/stamped_legoloam_odom_estimate.txt $result_path/traj/stamped_legoloam_odom_estimate{}.txt'.format(trial)
            os.system(command)
            command = 'mv $result_path/traj/stamped_legoloam_map_estimate.txt $result_path/traj/stamped_legoloam_map_estimate{}.txt'.format(trial)
            os.system(command)                        

        command = 'cp $result_path/traj/stamped_groundtruth{}.txt $result_path/traj/stamped_groundtruth.txt'.format(0)
        os.system(command)
        command = 'cp ~/catkin_ws/src/localization/M-LOAM/estimator/config/{} {}'.format(yaml_name, os.environ['result_path'])
        os.system(command)
        # command = 'cp /tmp/mloam_mapping_surf_cloud.pcd {}'.format(os.environ['result_path'])
        # os.system(command)
        # command = 'cp /tmp/mloam_mapping_surf_cloud_wo_ua.pcd {}'.format(os.environ['result_path'])
        # os.system(command)
        # command = 'cp /tmp/aloam_mapping.pcd {}'.format(os.environ['result_path'])
        # os.system(command)

# python2 run_mloam.py -sequence=SR -program=mc_eval \
#   -start_idx=0 -end_idx=4 -mc_trials=10
def mc_eval(start_idx, end_idx, mc_trials):
    for idx in range(start_idx, end_idx + 1):
        print('evaluate sequence: {}'.format(seq_name[idx]))
        os.environ['rpg_path'] = '{}/src/localization/rpg_trajectory_evaluation'.format(os.environ['CATKIN_WS'])
        os.environ['result_path'] = '{}/results/{}/monte_carlo_{}/'.format(os.environ['rpg_path'], platform, seq_name[idx])
        command = 'python2 $rpg_path/scripts/analyze_trajectory_single_mloam.py \
                   --recalculate_errors --est_type {} \
                   --compare $result_path/traj \
                   --mul_trials={}'.format(est_type, mc_trials)
        os.system(command)    

# python2 run_mloam.py -sequence=RHD -program=inject_ext_uct_test \
#   -start_idx=0 -end_idx=0 -ext_level=ref
def inject_ext_uct_test(start_idx, end_idx, ext_level):
    for idx in range(start_idx, end_idx + 1):
        print('testing sequence: {}'.format(seq_name[idx]))
        os.environ['data_path'] = '{}/lidar_calibration/mloam_dataset/{}.bag'.format(os.environ['DATA_PATH'], seq_name[idx])
        os.environ['rpg_path'] = '{}/src/localization/rpg_trajectory_evaluation'.format(os.environ['CATKIN_WS'])
        os.environ['result_path'] = '{}/results/{}/inject_ext_uct_{}/'.format(os.environ['rpg_path'], platform, seq_name[idx])
        command = 'mkdir -p $result_path/gf_pcd $result_path/traj $result_path/time \
                            $result_path/pose_graph $result_path/others $result_path/gf_pcd'
        os.system(command)
        command = 'bash {}'.format(seq_main_name)
        os.system(command)
        command = 'cp $result_path/traj/stamped_groundtruth.txt $result_path/traj/stamped_groundtruth_{}.txt'.format(ext_level)
        os.system(command)
        command = 'mv $result_path/traj/stamped_mloam_odom_estimate_1.000000.txt $result_path/traj/stamped_mloam_odom_estimate_1.000000_{}.txt'.format(ext_level)
        os.system(command)
        command = 'mv $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_0_{}.txt'.format(ext_level)
        os.system(command)
        command = 'mv $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_0_{}.txt'.format(ext_level)
        os.system(command)
        command = 'mv $result_path/traj/stamped_aloam_odom_estimate.txt $result_path/traj/stamped_aloam_odom_estimate_{}.txt'.format(ext_level)
        os.system(command)
        command = 'mv $result_path/traj/stamped_aloam_map_estimate.txt $result_path/traj/stamped_aloam_map_estimate_{}.txt'.format(ext_level)
        os.system(command)
        command = 'mv $result_path/traj/stamped_floam_map_estimate.txt $result_path/traj/stamped_floam_map_estimate_{}.txt'.format(ext_level)
        os.system(command)        
        command = 'mv $result_path/traj/stamped_legoloam_odom_estimate.txt $result_path/traj/stamped_legoloam_odom_estimate_{}.txt'.format(ext_level)
        os.system(command)                        
        command = 'mv $result_path/traj/stamped_legoloam_map_estimate.txt $result_path/traj/stamped_legoloam_map_estimate_{}.txt'.format(ext_level)
        os.system(command)
        command = 'cp ~/catkin_ws/src/localization/M-LOAM/estimator/config/{} {}/config_{}.yaml'.format(yaml_name, os.environ['result_path'], ext_level)
        os.system(command)        
        command = 'cp /tmp/mloam_mapping_surf_cloud.pcd {}/mloam_mapping_{}.pcd'.format(os.environ['result_path'], ext_level)
        os.system(command)
        command = 'cp /tmp/mloam_mapping_surf_cloud_wo_ua.pcd {}/mloam_mapping_wo_ua_{}.pcd'.format(os.environ['result_path'], ext_level)
        os.system(command)
        command = 'cp /tmp/aloam_mapping.pcd {}/aloam_mapping_{}.pcd'.format(os.environ['result_path'], ext_level)
        os.system(command)                           
        command = 'cp /tmp/legoloam_map_surf_cloud.pcd {}/legoloam_mapping_{}.pcd'.format(os.environ['result_path'], ext_level)
        os.system(command)                                

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='run mloam sr test')
    parser.add_argument('-program', help='debug_test, debug_eval, \
        calib_test, single_test, single_eval, mc_test, mc_eval, inject_ext_uct_test')
    parser.add_argument('-sequence', help='SR, RHD, RV, OR')
    parser.add_argument('-start_idx', type=int, help='idx')
    parser.add_argument('-end_idx', type=int, help='idx')
    parser.add_argument('-mc_trials', type=int, help='mc_trials=n')
    parser.add_argument('-ext_level', help='gt, ref, cad, ini, inj')
    parser.add_argument('-est_type', help='xx_xx')
    args = parser.parse_args()

    if args.sequence == 'SR':
        seq_name = ['SR01', 'SR02', 'SR03', 'SR04', 'SR05']
        platform = 'simu_jackal_mloam'
        seq_main_name = 'sr_main.sh'
        yaml_name = 'config_simu_jackel.yaml'
    elif args.sequence == 'RHD':
        seq_name = ['RHD02lab', 'RHD03garden', 'RHD04building', 'RHD06calib_filter']
        platform = 'handheld'
        seq_main_name = 'rhd_main.sh'
        yaml_name = 'config_handheld.yaml'
    elif args.sequence == 'RV':
        seq_name = ['RV00_calib', 'RV01', 'RV02']
        platform = 'real_vehicle/pingshan'
        seq_main_name = 'rv_pingshan_main.sh'
        yaml_name = 'config_realvehicle_hercules.yaml'  
    elif args.sequence == 'OR':
        seq_name = ['OR01', 'OR02', 'OR03', 'OR04', 'OR05', 'OR06']
        platform = 'real_vehicle/oxford'
        seq_main_name = 'rv_oxford_main.sh'
        yaml_name = 'config_realvehicle_oxford.yaml'
    elif args.sequence == 'KITTI':
        seq_name = ['KITTI00', 'KITTI03', 'KITTI04', 'KITTI05', 'KITTI06', 'KITTI07', 'KITTI09']
        platform = 'real_vehicle/kitti'
        seq_main_name = 'rv_kitti_main.sh'
        yaml_name = 'config_realvehicle_kitti.yaml'        

    if len(seq_name) < args.end_idx:
        print('exit! end_idx is too large: {} > {}'.format(args.end_idx, len(seq_name)))
        sys.exit(0)

    if args.start_idx > args.end_idx:
        print('exit! start_idx > end_idx: {} > {}'.format(args.start_idx, args.end_idx))
        sys.exit(0)      

    if args.program == 'calib_test':
        calib_test(args.start_idx, args.end_idx)
    elif args.program == 'debug_test':
        debug_test(args.start_idx)
    elif args.program == 'debug_eval':
        if not args.est_type:
            print('exit! no est_type')
            sys.exit(0)
        est_list = args.est_type.split(',')
        for str in est_list:
            est_type += str + ' '
        print('eval method: {}'.format(est_type))
        debug_eval()
    elif args.program == 'single_test':
        single_test(args.start_idx, args.end_idx)
    elif args.program == 'single_eval':
        est_list = args.est_type.split(',')
        for str in est_list:
            est_type += str + ' '
        print('eval method: {}'.format(est_type))
        single_eval(args.start_idx, args.end_idx)        
    elif args.program == 'mc_test':
        mc_test(args.start_idx, args.end_idx, args.mc_trials)
    elif args.program == 'mc_eval':
        est_list = args.est_type.split(',')
        for str in est_list:
            est_type += str + ' '
        print('eval method: {}'.format(est_type))
        mc_eval(args.start_idx, args.end_idx, args.mc_trials)
    elif args.program == 'inject_ext_uct_test':
        inject_ext_uct_test(args.start_idx, args.end_idx, args.ext_level)


