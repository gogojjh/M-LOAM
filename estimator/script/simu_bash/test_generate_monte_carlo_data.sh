# !/bin/bash

export data_path=$DATA_PATH/lidar_calibration/mloam_dataset/
export std=0.05

mkdir -p $data_path/SR_monte_carlo/group_1
rosrun mloam_test test_generate_noisy_bag $data_path/SR01.bag $data_path/SR_monte_carlo/group_1/SR01.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR02.bag $data_path/SR_monte_carlo/group_1/SR02.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR03.bag $data_path/SR_monte_carlo/group_1/SR03.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR04.bag $data_path/SR_monte_carlo/group_1/SR04.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR05.bag $data_path/SR_monte_carlo/group_1/SR05.bag $std
echo "Finish group_1 generation"

mkdir -p $data_path/SR_monte_carlo/group_2
rosrun mloam_test test_generate_noisy_bag $data_path/SR01.bag $data_path/SR_monte_carlo/group_2/SR01.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR02.bag $data_path/SR_monte_carlo/group_2/SR02.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR03.bag $data_path/SR_monte_carlo/group_2/SR03.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR04.bag $data_path/SR_monte_carlo/group_2/SR04.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR05.bag $data_path/SR_monte_carlo/group_2/SR05.bag $std
echo "Finish group_2 generation"

mkdir -p $data_path/SR_monte_carlo/group_3
rosrun mloam_test test_generate_noisy_bag $data_path/SR01.bag $data_path/SR_monte_carlo/group_3/SR01.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR02.bag $data_path/SR_monte_carlo/group_3/SR02.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR03.bag $data_path/SR_monte_carlo/group_3/SR03.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR04.bag $data_path/SR_monte_carlo/group_3/SR04.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR05.bag $data_path/SR_monte_carlo/group_3/SR05.bag $std
echo "Finish group_3 generation"

mkdir -p $data_path/SR_monte_carlo/group_4
rosrun mloam_test test_generate_noisy_bag $data_path/SR01.bag $data_path/SR_monte_carlo/group_4/SR01.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR02.bag $data_path/SR_monte_carlo/group_4/SR02.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR03.bag $data_path/SR_monte_carlo/group_4/SR03.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR04.bag $data_path/SR_monte_carlo/group_4/SR04.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR05.bag $data_path/SR_monte_carlo/group_4/SR05.bag $std
echo "Finish group_4 generation"

mkdir -p $data_path/SR_monte_carlo/group_5
rosrun mloam_test test_generate_noisy_bag $data_path/SR01.bag $data_path/SR_monte_carlo/group_5/SR01.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR02.bag $data_path/SR_monte_carlo/group_5/SR02.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR03.bag $data_path/SR_monte_carlo/group_5/SR03.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR04.bag $data_path/SR_monte_carlo/group_5/SR04.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR05.bag $data_path/SR_monte_carlo/group_5/SR05.bag $std
echo "Finish group_5 generation"

mkdir -p $data_path/SR_monte_carlo/group_6
rosrun mloam_test test_generate_noisy_bag $data_path/SR01.bag $data_path/SR_monte_carlo/group_6/SR01.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR02.bag $data_path/SR_monte_carlo/group_6/SR02.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR03.bag $data_path/SR_monte_carlo/group_6/SR03.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR04.bag $data_path/SR_monte_carlo/group_6/SR04.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR05.bag $data_path/SR_monte_carlo/group_6/SR05.bag $std
echo "Finish group_6 generation"

mkdir -p $data_path/SR_monte_carlo/group_7
rosrun mloam_test test_generate_noisy_bag $data_path/SR01.bag $data_path/SR_monte_carlo/group_7/SR01.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR02.bag $data_path/SR_monte_carlo/group_7/SR02.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR03.bag $data_path/SR_monte_carlo/group_7/SR03.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR04.bag $data_path/SR_monte_carlo/group_7/SR04.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR05.bag $data_path/SR_monte_carlo/group_7/SR05.bag $std
echo "Finish group_7 generation"

mkdir -p $data_path/SR_monte_carlo/group_8
rosrun mloam_test test_generate_noisy_bag $data_path/SR01.bag $data_path/SR_monte_carlo/group_8/SR01.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR02.bag $data_path/SR_monte_carlo/group_8/SR02.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR03.bag $data_path/SR_monte_carlo/group_8/SR03.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR04.bag $data_path/SR_monte_carlo/group_8/SR04.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR05.bag $data_path/SR_monte_carlo/group_8/SR05.bag $std
echo "Finish group_8 generation"

mkdir -p $data_path/SR_monte_carlo/group_9
rosrun mloam_test test_generate_noisy_bag $data_path/SR01.bag $data_path/SR_monte_carlo/group_9/SR01.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR02.bag $data_path/SR_monte_carlo/group_9/SR02.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR03.bag $data_path/SR_monte_carlo/group_9/SR03.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR04.bag $data_path/SR_monte_carlo/group_9/SR04.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR05.bag $data_path/SR_monte_carlo/group_9/SR05.bag $std
echo "Finish group_9 generation"

mkdir -p $data_path/SR_monte_carlo/group_10
rosrun mloam_test test_generate_noisy_bag $data_path/SR01.bag $data_path/SR_monte_carlo/group_10/SR01.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR02.bag $data_path/SR_monte_carlo/group_10/SR02.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR03.bag $data_path/SR_monte_carlo/group_10/SR03.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR04.bag $data_path/SR_monte_carlo/group_10/SR04.bag $std
rosrun mloam_test test_generate_noisy_bag $data_path/SR05.bag $data_path/SR_monte_carlo/group_10/SR05.bag $std
echo "Finish group_10 generation"

