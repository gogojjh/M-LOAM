# !/bin/bash

export data_path=$DATA_PATH/lidar_calibration/mloam_dataset/RV00_calib.bag
export rpg_path=$CATKIN_WS/src/localization/rpg_trajectory_evaluation
export result_path=$rpg_path/results/real_vehicle/pingshan/RV00/
export data_source=bag
export delta_idx=1
export start_idx=1
export end_idx=2000
mkdir -p $result_path/gf_pcd
mkdir -p $result_path/traj
mkdir -p $result_path/time
mkdir -p $result_path/pose_graph
mkdir -p $result_path/others

roslaunch mloam mloam_realvehicle_hercules.launch \
    run_mloam:=true \
    run_mloam_mapping:=false \
    result_save:=true \
    bag_file:=$data_path \
    data_source:=$data_source \
    output_path:=$result_path
