# !/bin/bash

export data_path=$DATA_PATH/lidar_calibration/mloam_dataset/RHD06calib_filter.bag
export rpg_path=$CATKIN_WS/src/localization/rpg_trajectory_evaluation
export result_path=$rpg_path/results/handheld/calib_diff_initial/wo_initialization/
mkdir -p $result_path/gf_pcd
mkdir -p $result_path/traj
mkdir -p $result_path/time
mkdir -p $result_path/pose_graph
mkdir -p $result_path/others\

roslaunch mloam mloam_handheld.launch \
    run_mloam:=true \
    run_mloam_mapping:=false \
    result_save:=true \
    bag_file:=$data_path \
    output_path:=$result_path
