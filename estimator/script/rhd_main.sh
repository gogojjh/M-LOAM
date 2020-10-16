# !/bin/bash

# ours
# roslaunch mloam mloam_handheld.launch \
#     run_mloam:=true \
#     run_mloam_mapping:=true \
#     with_ua:=true \
#     gf_method:=wo_gf \
#     gf_ratio_ini:=1.0 \
#     result_save:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
# sleep 5

roslaunch mloam mloam_realvehicle_handheld.launch \
    run_mloam:=true \
    run_mloam_mapping:=true \
    with_ua:=true \
    gf_method:=gd_float \
    gf_ratio_ini:=0.5 \
    result_save:=true \
    bag_file:=$data_path \
    output_path:=$result_path
# sleep 5

# # without uncertainty-aware
# roslaunch mloam mloam_handheld.launch \
#     run_mloam:=true \
#     run_mloam_mapping:=true \
#     with_ua:=false \
#     gf_method:=wo_gf \
#     gf_ratio_ini:=1.0 \
#     result_save:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path
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
