# !/bin/bash

roslaunch mloam mloam_realvehicle_kitti.launch \
    run_mloam:=true \
    run_mloam_mapping:=false \
    result_save:=true \
    gf_method:=gd_float \
    gf_ratio_ini:=0.2 \
    loss_mode:=gmc \
    gnc:=true \
    bag_file:=$data_path \
    output_path:=$result_path
sleep 5

# ## ours
# roslaunch mloam mloam_realvehicle_kitti.launch \
#     run_mloam:=true \
#     result_save:=true \
#     gf_method:=gd_float \
#     gf_ratio_ini:=0.2 \
#     loss_mode:=gmc \
#     gnc:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 5

# ## different feature ratio
# roslaunch mloam mloam_realvehicle_kitti.launch \
#     run_mloam:=true \
#     result_save:=true \
#     gf_method:=wo_gf \
#     gf_ratio_ini:=1.0 \
#     loss_mode:=gmc \
#     gnc:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 5

# ### different sampling
# roslaunch mloam mloam_realvehicle_kitti.launch \
#     run_mloam:=true \
#     result_save:=true \
#     gf_method:=rnd \
#     gf_ratio_ini:=0.2 \
#     loss_mode:=gmc \
#     gnc:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 5

# roslaunch mloam mloam_realvehicle_kitti.launch \
#     run_mloam:=true \
#     result_save:=true \
#     gf_method:=fps \
#     gf_ratio_ini:=0.2 \
#     loss_mode:=gmc \
#     gnc:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 5

# roslaunch mloam mloam_realvehicle_kitti.launch \
#     run_mloam:=true \
#     result_save:=true \
#     gf_method:=gd_fix \
#     gf_ratio_ini:=0.2 \
#     loss_mode:=gmc \
#     gnc:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 5

# ### different loss
# roslaunch mloam mloam_realvehicle_kitti.launch \
#     run_mloam:=true \
#     result_save:=true \
#     gf_method:=gd_float \
#     gf_ratio_ini:=0.2 \
#     loss_mode:=gmc \
#     gnc:=false \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 5

# roslaunch mloam mloam_realvehicle_kitti.launch \
#     run_mloam:=true \
#     result_save:=true \
#     gf_method:=gd_float \
#     gf_ratio_ini:=0.2 \
#     loss_mode:=huber \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 5

# ## different LiDAR SLAM
# roslaunch mloam mloam_realvehicle_kitti.launch \
#     run_mloam:=false \
#     run_aloam:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 5

# roslaunch mloam mloam_realvehicle_kitti.launch \
#     run_mloam:=false \
#     run_floam:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 5

# roslaunch mloam mloam_realvehicle_kitti.launch \
#     run_mloam:=false \
#     run_legoloam:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 5

# ## evaluation
# python2 $rpg_path/scripts/analyze_trajectory_single_mloam.py \
# --recalculate_errors \
# --est_type \
#     M-LOAM-gd-float-0.2-gmc-gnc \
#     M-LOAM-gd-float-1.0-gmc-gnc \
#     M-LOAM-rnd-0.2-gmc-gnc M-LOAM-fps-0.2-gmc-gnc M-LOAM-gd-fix-0.2-gmc-gnc \
#     M-LOAM-gd-float-0.2-gmc M-LOAM-gd-float-0.2-huber \
#     F-LOAM LEGO-LOAM \
# --compare \
#     $result_path/traj

