# !/bin/bash

export rpg_path=$CATKIN_WS/src/localization/rpg_trajectory_evaluation
export result_path=$rpg_path/results/simu_jackal_mloam/monte_carlo_SR05/
mkdir -p $result_path/gf_pcd
mkdir -p $result_path/traj
mkdir -p $result_path/time
mkdir -p $result_path/pose_graph
mkdir -p $result_path/others

echo "mc_trial 1"
export data_path=$DATA_PATH/lidar_calibration/mloam_dataset/SR_monte_carlo/group_1/SR05.bag
bash test_main_monte_carlo.sh
cp $result_path/traj/stamped_groundtruth.txt $result_path/traj/stamped_groundtruth1.txt
cp $result_path/traj/stamped_mloam_odom_estimate_1.000000.txt $result_path/traj/stamped_mloam_odom_estimate_1.0000001.txt
cp $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_01.txt
cp $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_01.txt
cp $result_path/traj/stamped_aloam_odom_estimate.txt $result_path/traj/stamped_aloam_odom_estimate1.txt
cp $result_path/traj/stamped_aloam_map_estimate.txt $result_path/traj/stamped_aloam_map_estimate1.txt
cp $result_path/traj/stamped_floam_map_estimate.txt $result_path/traj/stamped_floam_map_estimate1.txt

echo "mc_trial 2"
export data_path=$DATA_PATH/lidar_calibration/mloam_dataset/SR_monte_carlo/group_2/SR05.bag
bash test_main_monte_carlo.sh
mv $result_path/traj/stamped_groundtruth.txt $result_path/traj/stamped_groundtruth2.txt
mv $result_path/traj/stamped_mloam_odom_estimate_1.000000.txt $result_path/traj/stamped_mloam_odom_estimate_1.0000002.txt
mv $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_02.txt
mv $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_02.txt
mv $result_path/traj/stamped_aloam_odom_estimate.txt $result_path/traj/stamped_aloam_odom_estimate2.txt
mv $result_path/traj/stamped_aloam_map_estimate.txt $result_path/traj/stamped_aloam_map_estimate2.txt
mv $result_path/traj/stamped_floam_map_estimate.txt $result_path/traj/stamped_floam_map_estimate2.txt

echo "mc_trial 3"
export data_path=$DATA_PATH/lidar_calibration/mloam_dataset/SR_monte_carlo/group_3/SR05.bag
bash test_main_monte_carlo.sh
mv $result_path/traj/stamped_groundtruth.txt $result_path/traj/stamped_groundtruth3.txt
mv $result_path/traj/stamped_mloam_odom_estimate_1.000000.txt $result_path/traj/stamped_mloam_odom_estimate_1.0000003.txt
mv $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_03.txt
mv $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_03.txt
mv $result_path/traj/stamped_aloam_odom_estimate.txt $result_path/traj/stamped_aloam_odom_estimate3.txt
mv $result_path/traj/stamped_aloam_map_estimate.txt $result_path/traj/stamped_aloam_map_estimate3.txt
mv $result_path/traj/stamped_floam_map_estimate.txt $result_path/traj/stamped_floam_map_estimate3.txt

echo "mc_trial 4"
export data_path=$DATA_PATH/lidar_calibration/mloam_dataset/SR_monte_carlo/group_4/SR05.bag
bash test_main_monte_carlo.sh
mv $result_path/traj/stamped_groundtruth.txt $result_path/traj/stamped_groundtruth4.txt
mv $result_path/traj/stamped_mloam_odom_estimate_1.000000.txt $result_path/traj/stamped_mloam_odom_estimate_1.0000004.txt
mv $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_04.txt
mv $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_04.txt
mv $result_path/traj/stamped_aloam_odom_estimate.txt $result_path/traj/stamped_aloam_odom_estimate4.txt
mv $result_path/traj/stamped_aloam_map_estimate.txt $result_path/traj/stamped_aloam_map_estimate4.txt
mv $result_path/traj/stamped_floam_map_estimate.txt $result_path/traj/stamped_floam_map_estimate4.txt

echo "mc_trial 5"
export data_path=$DATA_PATH/lidar_calibration/mloam_dataset/SR_monte_carlo/group_5/SR05.bag
bash test_main_monte_carlo.sh
mv $result_path/traj/stamped_groundtruth.txt $result_path/traj/stamped_groundtruth5.txt
mv $result_path/traj/stamped_mloam_odom_estimate_1.000000.txt $result_path/traj/stamped_mloam_odom_estimate_1.0000005.txt
mv $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_05.txt
mv $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_05.txt
mv $result_path/traj/stamped_aloam_odom_estimate.txt $result_path/traj/stamped_aloam_odom_estimate5.txt
mv $result_path/traj/stamped_aloam_map_estimate.txt $result_path/traj/stamped_aloam_map_estimate5.txt
mv $result_path/traj/stamped_floam_map_estimate.txt $result_path/traj/stamped_floam_map_estimate5.txt

echo "mc_trial 6"
export data_path=$DATA_PATH/lidar_calibration/mloam_dataset/SR_monte_carlo/group_6/SR05.bag
bash test_main_monte_carlo.sh
mv $result_path/traj/stamped_groundtruth.txt $result_path/traj/stamped_groundtruth6.txt
mv $result_path/traj/stamped_mloam_odom_estimate_1.000000.txt $result_path/traj/stamped_mloam_odom_estimate_1.0000006.txt
mv $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_06.txt
mv $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_06.txt
mv $result_path/traj/stamped_aloam_odom_estimate.txt $result_path/traj/stamped_aloam_odom_estimate6.txt
mv $result_path/traj/stamped_aloam_map_estimate.txt $result_path/traj/stamped_aloam_map_estimate6.txt
mv $result_path/traj/stamped_floam_map_estimate.txt $result_path/traj/stamped_floam_map_estimate6.txt

echo "mc_trial 7"
export data_path=$DATA_PATH/lidar_calibration/mloam_dataset/SR_monte_carlo/group_7/SR05.bag
bash test_main_monte_carlo.sh
mv $result_path/traj/stamped_groundtruth.txt $result_path/traj/stamped_groundtruth7.txt
mv $result_path/traj/stamped_mloam_odom_estimate_1.000000.txt $result_path/traj/stamped_mloam_odom_estimate_1.0000007.txt
mv $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_07.txt
mv $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_07.txt
mv $result_path/traj/stamped_aloam_odom_estimate.txt $result_path/traj/stamped_aloam_odom_estimate7.txt
mv $result_path/traj/stamped_aloam_map_estimate.txt $result_path/traj/stamped_aloam_map_estimate7.txt
mv $result_path/traj/stamped_floam_map_estimate.txt $result_path/traj/stamped_floam_map_estimate7.txt

echo "mc_trial 8"
export data_path=$DATA_PATH/lidar_calibration/mloam_dataset/SR_monte_carlo/group_8/SR05.bag
bash test_main_monte_carlo.sh
mv $result_path/traj/stamped_groundtruth.txt $result_path/traj/stamped_groundtruth8.txt
mv $result_path/traj/stamped_mloam_odom_estimate_1.000000.txt $result_path/traj/stamped_mloam_odom_estimate_1.0000008.txt
mv $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_08.txt
mv $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_08.txt
mv $result_path/traj/stamped_aloam_odom_estimate.txt $result_path/traj/stamped_aloam_odom_estimate8.txt
mv $result_path/traj/stamped_aloam_map_estimate.txt $result_path/traj/stamped_aloam_map_estimate8.txt
mv $result_path/traj/stamped_floam_map_estimate.txt $result_path/traj/stamped_floam_map_estimate8.txt

echo "mc_trial 9"
export data_path=$DATA_PATH/lidar_calibration/mloam_dataset/SR_monte_carlo/group_9/SR05.bag
bash test_main_monte_carlo.sh
mv $result_path/traj/stamped_groundtruth.txt $result_path/traj/stamped_groundtruth9.txt
mv $result_path/traj/stamped_mloam_odom_estimate_1.000000.txt $result_path/traj/stamped_mloam_odom_estimate_1.0000009.txt
mv $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_09.txt
mv $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_09.txt
mv $result_path/traj/stamped_aloam_odom_estimate.txt $result_path/traj/stamped_aloam_odom_estimate9.txt
mv $result_path/traj/stamped_aloam_map_estimate.txt $result_path/traj/stamped_aloam_map_estimate9.txt
mv $result_path/traj/stamped_floam_map_estimate.txt $result_path/traj/stamped_floam_map_estimate9.txt

echo "mc_trial 10"
export data_path=$DATA_PATH/lidar_calibration/mloam_dataset/SR_monte_carlo/group_10/SR05.bag
bash test_main_monte_carlo.sh
mv $result_path/traj/stamped_groundtruth.txt $result_path/traj/stamped_groundtruth10.txt
mv $result_path/traj/stamped_mloam_odom_estimate_1.000000.txt $result_path/traj/stamped_mloam_odom_estimate_1.00000010.txt
mv $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_estimate_wo_gf_1.000000_huber_010.txt
mv $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_0.txt $result_path/traj/stamped_mloam_map_wo_ua_estimate_wo_gf_1.000000_huber_010.txt
mv $result_path/traj/stamped_aloam_odom_estimate.txt $result_path/traj/stamped_aloam_odom_estimate10.txt
mv $result_path/traj/stamped_aloam_map_estimate.txt $result_path/traj/stamped_aloam_map_estimate10.txt
mv $result_path/traj/stamped_floam_map_estimate.txt $result_path/traj/stamped_floam_map_estimate10.txt

# evaluation
python2 $rpg_path/scripts/analyze_trajectory_single_mloam.py \
--recalculate_errors \
--est_type \
    M-LO \
    M-LOAM-wo-ua M-LOAM \
    A-LOAM F-LOAM \
--compare \
    $result_path/traj \
--mul_trials=10


