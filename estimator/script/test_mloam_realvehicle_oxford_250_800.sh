# !/bin/bash

# bag: 250-800
# roslaunch mloam mloam_realvehicle_oxford.launch \
# run_mloam:=true \
# result_save:=true \
# gf_ratio_ini:=1.0 \
# bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_250_800.bag 
# output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_250_800/
# # 
# sleep 2

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=false \
run_legoloam:=true \
bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_250_800.bag \
output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_250_800/
#
sleep 2

# roslaunch mloam mloam_realvehicle_oxford.launch \
# run_mloam:=false \
# run_aloam:=true \
# bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_250_800.bag \
# output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_250_800/
# # 
# sleep 2
