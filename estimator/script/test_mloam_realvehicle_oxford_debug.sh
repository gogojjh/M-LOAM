# !/bin/bash

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_method:=gd_float \
gf_ratio_ini:=0.2 \
gnc:=false \
debug_mode:=false \
loss_mode:=gmc \
bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_250_800.bag \
output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_debug/
sleep 5

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_method:=gd_float \
gf_ratio_ini:=0.2 \
gnc:=true \
debug_mode:=false \
loss_mode:=gmc \
bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_250_800.bag \
output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_debug/
sleep 5

# roslaunch mloam mloam_realvehicle_oxford.launch \
# run_mloam:=true \
# result_save:=true \
# gf_method:=gd_float \
# gf_ratio_ini:=0.4 \
# gnc:=false \
# debug_mode:=false \
# loss_mode:=huber \
# bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_250_800.bag \
# output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_debug/
# sleep 5

# roslaunch mloam mloam_realvehicle_oxford.launch \
# run_mloam:=true \
# result_save:=true \
# gf_method:=gd_float \
# gf_ratio_ini:=0.4 \
# gnc:=false \
# debug_mode:=false \
# loss_mode:=gmc \
# bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_250_800.bag \
# output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_debug/
# sleep 5

# roslaunch mloam mloam_realvehicle_oxford.launch \
# run_mloam:=true \
# result_save:=true \
# gf_method:=gd_float \
# gf_ratio_ini:=0.4 \
# gnc:=true \
# debug_mode:=false \
# loss_mode:=gmc \
# bag_file:=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_250_800.bag \
# output_path:=/home/jjiao/trajectory_results/real_vehicle/oxford/20190118_debug/
# sleep 5

python2 /home/jjiao/catkin_ws/src/localization/rpg_trajectory_evaluation/scripts/analyze_trajectory_single_mloam.py \
--est_type M-LOAM-gd-float-0.4-huber \
M-LOAM-gd-float-0.2-gmc M-LOAM-gd-float-0.2-gmc-gnc \
M-LOAM-gd-float-0.4-gmc M-LOAM-gd-float-0.4-gmc-gnc \
LEGO-LOAM \
--compare /home/jjiao/catkin_ws/src/localization/rpg_trajectory_evaluation/results/real_vehicle/oxford/20190118_debug/
