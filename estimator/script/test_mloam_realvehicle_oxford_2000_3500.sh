# !/bin/bash

# bag: 2000-3500
roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_method:=gd_float \
gf_ratio_ini:=0.2 \
bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_2000_3500.bag \
output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_2000_3500/
sleep 10

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_method:=gd_float \
gf_ratio_ini:=0.4 \
bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_2000_3500.bag \
output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_2000_3500/
sleep 10

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_method:=gd_float \
gf_ratio_ini:=0.6 \
bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_2000_3500.bag \
output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_2000_3500/
sleep 10

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_method:=gd_float \
gf_ratio_ini:=0.8 \
bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_2000_3500.bag \
output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_2000_3500/
sleep 10

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_method:=gd_float \
gf_ratio_ini:=1.0 \
bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_2000_3500.bag \
output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_2000_3500/
sleep 10

# roslaunch mloam mloam_realvehicle_oxford.launch \
# run_mloam:=false \
# run_legoloam:=true \
# bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_2000_3500.bag \
# output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_2000_3500/
# sleep 10

# roslaunch mloam mloam_realvehicle_oxford.launch \
# run_mloam:=false \
# run_aloam:=true \
# bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_2000_3500.bag \
# output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_2000_3500/
# sleep 10
