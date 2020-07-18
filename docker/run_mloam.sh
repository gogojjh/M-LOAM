#!/bin/bash

docker run \
    -it \
    -v ${CATKIN_WS}:/usr/app/catkin_ws \
    -v ${DATA_PATH}:/usr/app/dataset \
    -p 8050:8888 \
    --name jjiao_mloam \
    ros:melodic-mloam /bin/bash

