# !/bin/bash

# bag: 1-800
roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_ratio_ini:=0.2 \
bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_1_800.bag \
output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_1_800/
sleep 10

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_ratio_ini:=0.4 \
bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_1_800.bag \
output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_1_800/
sleep 10

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_ratio_ini:=0.6 \
bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_1_800.bag \
output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_1_800/
sleep 10

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_ratio_ini:=0.8 \
bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_1_800.bag \
output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_1_800/
sleep 10

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_ratio_ini:=1.0 \
bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_1_800.bag \
output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_1_800/
sleep 10

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=false \
run_legoloam:=true \
bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_1_800.bag \
output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_1_800/
sleep 10

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=false \
run_aloam:=true \
bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_1_800.bag \
output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_1_800/
sleep 10
