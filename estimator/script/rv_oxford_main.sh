!/bin/bash

################## M-LOAM-GF with different good feature ratio
roslaunch mloam mloam_realvehicle_oxford.launch \
    run_mloam:=true \
    run_mloam_mapping:=true \
    with_ua:=true \
    gf_method:=wo_gf \
    gf_ratio_ini:=1.0 \
    result_save:=true \
    bag_file:=$data_path \
    output_path:=$result_path \
    bag_record:=true
sleep 20

# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=true \
#     run_mloam_mapping:=true \
#     gf_method:=gd_float \
#     gf_ratio_ini:=0.8 \
#     result_save:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path \
#     bag_record:=true
# sleep 20

# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=true \
#     run_mloam_mapping:=true \
#     gf_method:=gd_float \
#     gf_ratio_ini:=0.6 \
#     result_save:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path \
#     bag_record:=true
# sleep 20

# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=true \
#     run_mloam_mapping:=true \
#     with_ua:=true \
#     gf_method:=gd_float \
#     gf_ratio_ini:=0.4 \
#     result_save:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path \
#     bag_record:=true
# sleep 20

# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=true \
#     run_mloam_mapping:=true \
#     gf_method:=gd_float \
#     gf_ratio_ini:=0.2 \
#     result_save:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path \
#     bag_record:=true
# sleep 20

# ################## different point selection methods
roslaunch mloam mloam_realvehicle_oxford.launch \
    run_mloam:=true \
    run_mloam_mapping:=true \
    with_ua:=true \
    gf_method:=rnd \
    gf_ratio_ini:=0.2 \
    result_save:=true \
    bag_file:=$data_path \
    output_path:=$result_path \
    bag_record:=true
sleep 20

# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=true \
#     run_mloam_mapping:=true \
#     gf_method:=fps \
#     gf_ratio_ini:=0.2 \
#     result_save:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path \
#     bag_record:=true
# sleep 20

# roslaunch mloam mloam_realvehicle_oxford.launch \
#     run_mloam:=true \
#     run_mloam_mapping:=true \
#     gf_method:=gd_fix \
#     gf_ratio_ini:=0.2 \
#     result_save:=true \
#     bag_file:=$data_path \
#     output_path:=$result_path \
#     bag_record:=true
# sleep 20

################## different LiDAR SLAM
roslaunch mloam mloam_realvehicle_oxford.launch \
    run_mloam:=false \
    run_aloam:=true \
    bag_file:=$data_path \
    output_path:=$result_path \
    bag_record:=true
sleep 20

roslaunch mloam mloam_realvehicle_oxford.launch \
    run_mloam:=false \
    run_floam:=true \
    bag_file:=$data_path \
    output_path:=$result_path \
    bag_record:=true
sleep 20

roslaunch mloam mloam_realvehicle_oxford.launch \
    run_mloam:=false \
    run_legoloam:=true \
    bag_file:=$data_path \
    output_path:=$result_path \
    bag_record:=true
sleep 20

roslaunch mloam mloam_realvehicle_oxford.launch \
    run_mloam:=false \
    run_legoloam:=true \
    bag_file:=$data_path \
    output_path:=$result_path
sleep 5
