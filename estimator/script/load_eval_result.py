# ! /usr/bin/python2
# Usage 1: python run_sr_test.py -sequence=SR -program=mc_test -start_idx=0 -end_idx=4 -trials=10

# Usage 2: python load_eval_result.py \
# -path=/home/jjiao/catkin_ws/src/localization/rpg_trajectory_evaluation/results/real_vehicle/oxford/ \
# -sequence=OR01,OR02,OR03,OR04,OR05 -est_type=M-LOAM-gf-0.4,M-LOAM-rnd-0.4,M-LOAM-fps-0.4,\
# M-LOAM-gf-fix-0.4,M-LOAM-full,A-LOAM,F-LOAM,LEGO-LOAM -eval_type=single -err_type=ate,rpe

import sys
import os
import argparse
import numpy as np
import yaml

def main(path, sequences, est_types, eval_type, err_type):
    # print('method: {}'.format(' '.join(['{}'.format(s) for s in est_types])))
    if 'ate' in err_type:
        print('Loading ATE: {}'.format(sequences))
        for est_type in est_types:
            ate_rmse_trans = []
            ate_std_trans = []
            ate_rmse_rot = []
            ate_std_rot = []
            for seq in sequences:
                if eval_type == 'single':
                    eval_res_fn = os.path.join(path, seq, 'traj/saved_results', est_type, 'absolute_err_statistics_se3_-1.yaml')
                elif eval_type == 'mc':
                    eval_res_fn = os.path.join(path, seq, 'traj/saved_results', est_type, 'mt_abs_err_se3_-1.yaml')
                # print('- Process {}...'.format(eval_res_fn))
                with open(eval_res_fn, 'r') as f:
                    res = yaml.load(f, Loader=yaml.FullLoader)
                    ate_rmse_trans.append(res['trans']['rmse'])
                    ate_std_trans.append(res['trans']['std'])
                    ate_rmse_rot.append(res['rot']['rmse'])
                    ate_std_rot.append(res['rot']['std'])

            print('{};  {} (rmse trans)'.format(' & '.join(['{:.3f}\pm{:.3f}'.format(ate_rmse_trans[i], ate_std_trans[i]) for i in range(len(ate_rmse_trans))]), est_type))
            print('{};  {} (rmse rot)'.format(' & '.join(['{:.3f}\pm{:.3f}'.format(ate_rmse_rot[i], ate_std_rot[i]) for i in range(len(ate_rmse_rot))]), est_type))

    if 'rpe' in err_type:
        print('\nLoading RPE: {}'.format(sequences))
        for seq in sequences:
            print('\nseq: {}'.format(seq))
            str_meter = []
            for est_type in est_types:
                rpe_rmse_trans = []
                rpe_std_trans = []
                rpe_rmse_rot = []
                rpe_std_rot = []
                filename = os.listdir(os.path.join(path, seq, 'traj/saved_results', est_type))
                if eval_type == 'single':
                    filename = [f for f in filename if 'relative_error_statistics' in f]
                elif eval_type == 'mc':
                    filename = [f for f in filename if 'mt_rel_err' in f]
                filename.sort()
                str_meter = []
                for file in filename:
                    str = file.split('_')
                    str_meter.append(str[3])
                    eval_res_fn = os.path.join(path, seq, 'traj/saved_results', est_type, file)
                    with open(eval_res_fn, 'r') as f:
                        res = yaml.load(f, Loader=yaml.FullLoader)
                        rpe_rmse_trans.append(res['trans_perc']['rmse'])
                        rpe_std_trans.append(res['trans_perc']['std'])
                        rpe_rmse_rot.append(res['rot_deg_per_m']['rmse'])
                        rpe_std_rot.append(res['rot_deg_per_m']['std'])
                print('{};  {} (rpe rmse trans_perc)'.format(' & '.join(['{:.3f}\%\pm{:.3f}\%'.format(rpe_rmse_trans[i], rpe_std_trans[i]) for i in range(len(rpe_rmse_trans))]), est_type))
                # print('{} (rmse rot_deg_per_m)\n'.format(seq, ' & '.join(['{:.3f}\pm{:.3f}'.format(rpe_rmse_rot[i], rpe_std_rot[i]) for i in range(len(rpe_rmse_rot))])))                   
            print(' '.join(['{}m'.format(s) for s in str_meter]))

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='load evaluation results')
    parser.add_argument('-path', help='path to result')
    parser.add_argument('-sequence', help='SR01')
    parser.add_argument('-est_type', help='M-LOAM,A-LOAM')
    parser.add_argument('-eval_type', help='single or mc')
    parser.add_argument('-err_type', help='ate,rpe')
    args = parser.parse_args()
    est_types = args.est_type.split(',')
    sequences = args.sequence.split(',')
    err_type = args.err_type.split(',')

    main(args.path, sequences, est_types, args.eval_type, args.err_type)
