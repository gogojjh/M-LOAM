# car_rviz_model
　[Robosense](http://www.robosense.cn/) rviz RobotModel for Car.
<p align="center">
    <img src=".readme/demo.png" width="720px" alt=""/>
</p>

## How to build
　`git clone` into your ros workspace `ROS_WS`.
```bash
$ cd $(ROS_WS)/src/common
$ git clone https://github.com/LidarPerception/rviz_car_model.git

$ cd $(ROS_WS)/
$ catkin build -DCMAKE_BUILD_TYPE=Release
```


## How to use
+ roslaunch the demo.
    ```bash
    $ cd $(ROS_WS)
    $ source devel/setup.bash
    $ roslaunch rviz_car_model demo.launch
    ```
+ **use in your own project for visualization**.
    1. Setup your launch file.
        ```xml
        <!-- Load rviz_car_model before rviz -->
        <include file="$(find rviz_car_model)/launch/default.launch">
            <arg name="fixed_frame" value="base_link"/>
            <!-- x y z qx qy qz qw -->
            <arg name="transform" value="0 0 -0.98 0 0 0 1"/>
        </include>
        ```
    2. Add a **RobotModel** in rviz.


## Parameters
+ `fixed_frame`: **Fixed-frame** for rviz or any frame you want to fix the Car to, default is **base_link**.
+ `transform`: Coordinates of the Car in `fixed_frame`-Coordinate, in Quaternion form: _x y z qx qy qz qw_. default is **0 0 0 0 0 0 1**, no translation and no rotation.