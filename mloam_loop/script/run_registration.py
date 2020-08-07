#! /usr/bin/python3

import os

max_x = 21
max_y = 21
max_yaw = 91

# max_x = 6
# max_y = 6
# max_yaw = 31

for x in range(0, max_x, 5):
    for y in range(0, max_y, 5):
        for yaw in range(0, max_yaw, 30):
            path = '../data/loop/transform_data_{}_{}_{}/'.format(x, y, yaw)
            if not os.path.exists(path):
                continue
            print('Loading data from: {}'.format(path))
            command = 'rosrun mloam_loop test_icp {}'.format(path)
            os.system(command)
            command = 'rosrun mloam_loop test_fgr {}'.format(path)
            os.system(command)
            command = 'rosrun mloam_loop test_evaluation {}'.format(path)
            os.system(command)

