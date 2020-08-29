# !/bin/bash

# ours
roslaunch mloam mloam_realvehicle_hercules.launch \
    run_mloam:=true \
    run_mloam_mapping:=true \
    result_save:=true \
    data_path:=$data_path \
    data_source:=$data_source \
    delta_idx:=$delta_idx \
    start_idx:=$start_idx \
    end_idx:=$end_idx \
    output_path:=$result_path
# sleep 5

# # without uncertainty-aware
# roslaunch mloam mloam_realvehicle_hercules.launch \
#     run_mloam:=true \
#     run_mloam_mapping:=true \
#     with_ua:=false \
#     result_save:=true \
#     data_path:=$data_path \
#     data_source:=$data_source \
#     delta_idx:=$delta_idx \
#     start_idx:=$start_idx \
#     end_idx:=$end_idx \
#     output_path:=$result_path
# sleep 5

# # different LiDAR SLAM
# roslaunch mloam mloam_realvehicle_hercules.launch \
#     run_mloam:=false \
#     run_aloam:=true \
#     data_path:=$data_path \
#     data_source:=$data_source \
#     delta_idx:=$delta_idx \
#     start_idx:=$start_idx \
#     end_idx:=$end_idx \
#     output_path:=$result_path
# sleep 5

# roslaunch mloam mloam_realvehicle_hercules.launch \
#     run_mloam:=false \
#     run_floam:=true \
#     data_path:=$data_path \
#     data_source:=$data_source \
#     delta_idx:=$delta_idx \
#     start_idx:=$start_idx \
#     end_idx:=$end_idx \
#     output_path:=$result_path
# sleep 5

# roslaunch mloam mloam_realvehicle_hercules.launch \
#     run_mloam:=false \
#     run_legoloam:=true \
#     data_path:=$data_path \
#     data_source:=$data_source \
#     delta_idx:=$delta_idx \
#     start_idx:=$start_idx \
#     end_idx:=$end_idx \
#     output_path:=$result_path
# sleep 5

# evaluation
# python2 $rpg_path/scripts/analyze_trajectory_single_mloam.py \
# --recalculate_errors \
# --est_type \
#     M-LOAM-wo-ua M-LOAM \
#     A-LOAM F-LOAM LEGO-LOAM \
# --compare \
#     $result_path/traj

# python2 $rpg_path/scripts/analyze_trajectory_single_mloam.py \
# --est_type \
#     M-LOAM-wo-ua M-LOAM \
#     A-LOAM F-LOAM LEGO-LOAM \
# --compare \
#     $result_path/traj

