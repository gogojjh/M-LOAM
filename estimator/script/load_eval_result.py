# ! /usr/bin/python2
# Usage: python run_sr_test.py -sequence=SR -program=mc_test -start_idx=0 -end_idx=4 -trials=10

import sys
import os
import argparse
import numpy as np
import yaml

def main(path, sequences, est_types, eval_type):
    print('method: {}'.format(' '.join(['{}'.format(s) for s in est_types])))
    for seq in sequences:
        rmse_trans = []
        rmse_rot = []
        for est_type in est_types:
            if eval_type == 'single':
                eval_res_fn = os.path.join(path, seq, 'traj/saved_results', est_type, 'absolute_err_statistics_se3_-1.yaml')
            elif eval_type == 'mc':
                eval_res_fn = os.path.join(path, seq, 'traj/saved_results', est_type, 'mt_abs_err_se3_-1.yaml')
            # print('- Process {}...'.format(eval_res_fn))
            with open(eval_res_fn, 'r') as f:
                res = yaml.load(f, Loader=yaml.FullLoader)
                rmse_trans.append(res['trans']['rmse'])
                rmse_rot.append(res['rot']['rmse'])
        
        print('{}: {} (rmse trans)'.format(seq, ' & '.join(['{:.3f}'.format(r) for r in rmse_trans])))
        print('{}: {} (rmse rot)\n'.format(seq, ' & '.join(['{:.3f}'.format(r) for r in rmse_rot])))

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='load evaluation results')
    parser.add_argument('-path', help='path to result')
    parser.add_argument('-sequence', help='SR01')
    parser.add_argument('-est_type', help='M-LOAM,A-LOAM')
    parser.add_argument('-eval_type', help='single or mc')
    args = parser.parse_args()
    est_types = args.est_type.split(',')
    sequences = args.sequence.split(',')

    main(args.path, sequences, est_types, args.eval_type)
