# !/bin/bash

# ours
roslaunch mloam mloam_handheld.launch \
    run_mloam:=true \
    run_mloam_mapping:=true \
    result_save:=true \
    bag_file:=$data_path \
    output_path:=$result_path
# sleep 5

# without uncertainty-aware
# roslaunch mloam mloam_handheld.launch \
#    run_mloam:=true \
#    with_ua:=false \
#    result_save:=true \
#    bag_file:=$data_path \
#    output_path:=$result_path
# sleep 5

# different LiDAR SLAM
# roslaunch mloam mloam_handheld.launch \
#     run_mloam:=false \
#     run_aloam:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 5

# roslaunch mloam mloam_handheld.launch \
#     run_mloam:=false \
#     run_floam:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 5

# # evaluation
# python2 $rpg_path/scripts/analyze_trajectory_single_mloam.py \
# --recalculate_errors \
# --est_type \
#     M-LO \
#     M-LOAM-wo-ua M-LOAM \
#     A-LOAM F-LOAM \
# --compare \
#     $result_path/traj

