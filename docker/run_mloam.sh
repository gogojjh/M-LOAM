#!/bin/bash

# roscore &
# ROSCORE_PID=$!
# sleep 1

docker run \
    -it \
    -v ${CATKIN_WS}/src:/usr/app/catkin_ws/src \
    -v ${DATA_PATH}:/usr/app/dataset \
    -p 8050:8888 \
    --name mloam_batch_test \
    ros:melodic-mloam /bin/bash 

# docker run \
#     -it \
#     -v ${CATKIN_WS}/src:/usr/app/catkin_ws/src \
#     -v ${DATA_PATH}:/usr/app/dataset \
#     -p 8050:8888 \
#     --name mloam_batch_test \
#     ros:melodic-mloam /bin/bash -c \
#     "cd /usr/app/catkin_ws/; \
#     catkin config \
#         --cmake-args \
#             -DCMAKE_BUILD_TYPE=Release; \
#         catkin build; \
#         source devel/setup.bash;" 
