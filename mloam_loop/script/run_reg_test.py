#! /usr/bin/python3

import os
import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

matplotlib.use('TkAgg')

def generate_data(path):
    command = 'rosrun mloam_loop test_generate_data {}'.format(path)
    os.system(command)    

def run_registration(path, max_x, max_y, max_yaw):
    icp_r_error = []
    icp_t_error = []
    fgr_r_error = []
    fgr_t_error = []
    for x in range(0, max_x, 1):
        for y in range(0, max_y, 1):
            for yaw in range(0, max_yaw, 5):
                path2 = '{}transform_data_{}_{}_{}/'.format(path, x, y, yaw)
                if not os.path.exists(path2):
                    continue
                print('Loading data from: {}'.format(path2))
                command = 'rosrun mloam_loop test_icp {}'.format(path2)
                os.system(command)
                # command = 'rosrun mloam_loop test_fgr {}'.format(path2)
                # os.system(command)
                command = 'rosrun mloam_loop test_registration_error {}'.format(path2)
                os.system(command)

                file = open('{}transform_error.txt'.format(path2), 'r')
                icp_r_e, fgr_r_e = map(float, file.readline().split(' '))
                icp_t_e, fgr_t_e = map(float, file.readline().split(' '))
                file.close()
                icp_r_error.append(icp_r_e)
                icp_t_error.append(icp_t_e)
                fgr_r_error.append(fgr_r_e)
                fgr_t_error.append(fgr_t_e)
    
    frame = [i for i in range(len(icp_r_error))]
    plt.subplot(2, 1, 1)
    plt.plot(frame, icp_r_error, label='ICP')
    plt.plot(frame, fgr_r_error, label='FGR')
    plt.title('Rotation Error')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(frame, icp_t_error, label='ICP')
    plt.plot(frame, fgr_t_error, label='FGR')
    plt.title('Translation Error')
    plt.legend()

    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'run_reg_test')
    parser.add_argument('-path', help = 'path to data folder')
    parser.add_argument('-program', help = 'main program: generate_data, run_registration')
    parser.add_argument('-max_x', type=int, help = 'max_x')
    parser.add_argument('-max_y', type=int, help = 'max_y')
    parser.add_argument('-max_yaw', type=int, help = 'max_yaw')
    args = parser.parse_args()
    if args.program == 'generate_data':
        generate_data(args.path)
    elif args.program == 'run_registration':
        max_x = int(args.max_x)
        max_y = int(args.max_y)
        max_yaw = int(args.max_yaw)
        run_registration(args.path, max_x, max_y, max_yaw)

