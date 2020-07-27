# !/bin/bash

export data_path=$DATA_PATH/KITTI/odometry/2011_09_30_0018_filter.bag
export rpg_path=$CATKIN_WS/src/localization/rpg_trajectory_evaluation
export result_path=$rpg_path/results/real_vehicle/kitti/xxx/
mkdir -p $result_path/gf_pcd
mkdir -p $result_path/traj
mkdir -p $result_path/time
mkdir -p $result_path/others

bash test_main.sh