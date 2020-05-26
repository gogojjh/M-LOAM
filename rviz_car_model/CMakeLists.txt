cmake_minimum_required(VERSION 2.8.3)
project(rviz_car_model)

#############
## Install ##
#############

install(FILES
    README.md
    launch/demo.launch
    rviz/car_model/_51225ex1diffuse.jpeg
    rviz/car_model/_51225ex2diffuse.jpeg
    rviz/car_model/CAR_original.dae
    rviz/car_model/default.urdf
    rviz/demo.rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${PROJECT_NAME}
)