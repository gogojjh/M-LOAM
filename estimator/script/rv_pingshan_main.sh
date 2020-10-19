# !/bin/bash

export data_source=bag

# ours
roslaunch mloam mloam_realvehicle_hercules.launch \
    run_mloam:=true \
    run_mloam_mapping:=true \
    result_save:=true \
    bag_file:=$data_path \
    data_source:=$data_source \
    output_path:=$result_path
# sleep 10

# without uncertainty-aware
# roslaunch mloam mloam_realvehicle_hercules.launch \
#     run_mloam:=true \
#     run_mloam_mapping:=true \
#     with_ua:=false \
#     result_save:=true \
#     bag_file:=$data_path \
#     data_source:=$data_source \
#     output_path:=$result_path
# sleep 10

# # different LiDAR SLAM
# roslaunch mloam mloam_realvehicle_hercules.launch \
#     run_mloam:=false \
#     run_aloam:=true \
#     bag_file:=$data_path \
#     data_source:=$data_source \
#     output_path:=$result_path
# sleep 10

# # roslaunch mloam mloam_realvehicle_hercules.launch \
# #     run_mloam:=false \
# #     run_floam:=true \
# #     bag_file:=$data_path \
# #     data_source:=$data_source \
# #     output_path:=$result_path
# # sleep 10

# roslaunch mloam mloam_realvehicle_hercules.launch \
#     run_mloam:=false \
#     run_legoloam:=true \
#     bag_file:=$data_path \
#     data_source:=$data_source \
#     output_path:=$result_path
# sleep 10

