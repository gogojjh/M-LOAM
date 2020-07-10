# !/bin/bash

export data_path=/Monster/dataset/oxford_radar_dataset/2019-01-18-15-20-12-radar-oxford-10k/data_ds_800_2000.bag
export rpg_path=/home/jjiao/catkin_ws/src/localization/rpg_trajectory_evaluation
export result_path=$rpg_path/results/real_vehicle/oxford/20190118_800_2000/

# bag: 800-2000
roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_ratio_ini:=0.2 \
bag_file:=$data_path \
output_path:=$result_path
sleep 10

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_ratio_ini:=0.4 \
bag_file:=$data_path \
output_path:=$result_path
sleep 10

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_ratio_ini:=0.6 \
bag_file:=$data_path \
output_path:=$result_path
sleep 10

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_ratio_ini:=0.8 \
bag_file:=$data_path \
output_path:=$result_path
sleep 10

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=true \
result_save:=true \
gf_ratio_ini:=1.0 \
bag_file:=$data_path \
output_path:=$result_path
sleep 10

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=false \
run_legoloam:=true \
bag_file:=$data_path \
output_path:=$result_path
sleep 10

roslaunch mloam mloam_realvehicle_oxford.launch \
run_mloam:=false \
run_aloam:=true \
bag_file:=$data_path \
output_path:=$result_path
sleep 10

python2 $rpg_path/scripts/analyze_trajectory_single_mloam.py --est_type \
    M-LOAM-gf-0.2 M-LOAM-gf-0.4 M-LOAM-gf-0.6 M-LOAM-gf-0.8 M-LOAM-gf-1.0 A-LOAM LEGO-LOAM --compare \
    $result_path