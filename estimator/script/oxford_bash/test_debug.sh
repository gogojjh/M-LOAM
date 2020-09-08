# !/bin/bash

export data_path=$DATA_PATH/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_800_2000.bag
export rpg_path=$CATKIN_WS/src/localization/rpg_trajectory_evaluation
export result_path=$rpg_path/results/real_vehicle/oxford/20190118_debug/
mkdir -p $result_path/gf_pcd
mkdir -p $result_path/traj
mkdir -p $result_path/time
mkdir -p $result_path/pose_graph
mkdir -p $result_path/others

bash test_main.sh