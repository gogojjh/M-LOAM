# !/bin/bash

# ours
# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=true \
#     run_mloam_mapping:=true \
#     gf_method:=wo_gf \
#     gnc:=false \
#     loss_mode:=huber \
#     result_save:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 10

# # without uncertainty-aware
# roslaunch mloam mloam_realvehicle_oxford.launch \
#    run_mloam:=true \
#    with_ua:=false \
#    gf_method:=wo_gf \
#    gnc:=false \
#    loss_mode:=huber \
#    result_save:=true \
#    bag_file:=$data_path \
#    output_path:=$result_path
# sleep 10

# different LiDAR SLAM
# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=false \
#     run_aloam:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 10

# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=false \
#     run_floam:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 10

# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=false \
#     run_legoloam:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 10

# evaluation
python2 $rpg_path/scripts/analyze_trajectory_single_mloam.py \
--recalculate_errors \
--est_type \
    M-LOAM-one-lidar \
    M-LOAM-wo-ua M-LOAM \
    A-LOAM LEGO-LOAM \
--compare \
    $result_path/traj



#####################################################################################
## ours
# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=true \
#     result_save:=true \
#     gf_method:=gd_float \
#     gf_ratio_ini:=0.2 \
#     loss_mode:=gmc \
#     gnc:=false \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 10

# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=true \
#     result_save:=true \
#     gf_method:=gd_float \
#     gf_ratio_ini:=0.2 \
#     loss_mode:=huber \
#     gnc:=false \
#     bag_file:=$data_path \
#     output_path:=$result_path

# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=true \
#     result_save:=true \
#     gf_method:=gd_float \
#     gf_ratio_ini:=0.2 \
#     loss_mode:=huber \
#     gnc:=false \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 10

## different feature ratio
# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=true \
#     result_save:=true \
#     gf_method:=wo_gf \
#     gf_ratio_ini:=1.0 \
#     loss_mode:=gmc \
#     gnc:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 10

# ### different sampling
# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=true \
#     result_save:=true \
#     gf_method:=rnd \
#     gf_ratio_ini:=0.2 \
#     loss_mode:=gmc \
#     gnc:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 10

# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=true \
#     result_save:=true \
#     gf_method:=fps \
#     gf_ratio_ini:=0.2 \
#     loss_mode:=gmc \
#     gnc:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 10

# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=true \
#     result_save:=true \
#     gf_method:=gd_fix \
#     gf_ratio_ini:=0.2 \
#     loss_mode:=gmc \
#     gnc:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 10

## different loss
# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=true \
#     result_save:=true \
#     gf_method:=gd_float \
#     gf_ratio_ini:=0.2 \
#     loss_mode:=gmc \
#     gnc:=false \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 10

# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=true \
#     result_save:=true \
#     gf_method:=gd_float \
#     gf_ratio_ini:=0.2 \
#     loss_mode:=huber \
#     gnc:=false \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 10

## different LiDAR SLAM
# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=false \
#     run_aloam:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 10

# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=false \
#     run_floam:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 10

# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=false \
#     run_legoloam:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 10

# evaluation
# python2 $rpg_path/scripts/analyze_trajectory_single_mloam.py \
# --recalculate_errors \
# --est_type \
#     M-LOAM-gd-float-0.2-gmc-gnc \
#     M-LOAM-wo-gf-1.0-gmc-gnc \
#     M-LOAM-rnd-0.2-gmc-gnc M-LOAM-fps-0.2-gmc-gnc M-LOAM-gd-fix-0.2-gmc-gnc \
#     M-LOAM-gd-float-0.2-gmc M-LOAM-gd-float-0.2-huber \
#     F-LOAM LEGO-LOAM \
# --compare \
#     $result_path/traj

# python2 $rpg_path/scripts/analyze_trajectory_single_mloam.py \
# --recalculate_errors \
# --est_type \
#     M-LOAM-gd-float-0.2-huber \
#     LEGO-LOAM
# --compare \
#     $result_path/traj
