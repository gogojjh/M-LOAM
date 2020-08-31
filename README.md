# M-LOAM
### Multi-LiDAR Odometry and Mapping
M-LOAM is a robust system for multi-LiDAR extrinsic calibration, real-time odometry, and mapping. Without manual intervention, our
system can start with several extrinsic-uncalibrated LiDARs, automatically calibrate their extrinsics, and provide accurate poses as well as a globally consistent map.

**Authors:** 
[Jianhao Jiao](http://gogojjh.github.io), 
[Haoyang Ye](https://github.com/hyye),
[Yilong Zhu](https://scholar.google.com/citations?user=x8n6v2oAAAAJ&hl=zh-CN),
[Linxin Jiang](),
[Ming Liu](https://scholar.google.com/citations?user=CdV5LfQAAAAJ&hl=zh-CN)
from [RAM-LAB](https://www.ramlab.com)

### 1. Prerequisites
#### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

#### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

#### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).
> NOTE: <br>
> Starting with PCL-1.7 you need to define PCL_NO_PRECOMPILE before you include any PCL headers to include the templated algorithms as well.
> Choose the right linear solver: http://ceres-solver.org/solving_faqs.html

#### 1.4. **OpenMP**
```sudo apt install libomp-dev```
> NOTE: <br>
> tutorial: https://bisqwit.iki.fi/story/howto/openmp/#IntroductionToOpenmpInC <br>
> slide: https://www3.nd.edu/~zxu2/acms60212-40212-S12/Lec-11-01.pdf

#### 1.5. **Eigen3**
```sudo apt install libeigen-dev```
> NOTE: to prevent Eigen error: <br> 
> https://eigen.tuxfamily.org/dox/group__TopicUnalignedArrayAssert.html

#### 1.6. **GLOG, GFLAGS, GTEST**
> NOTE: installation <br>
> https://juejin.im/post/5dca40b9f265da4d226e397e

### 2. Build M-LOAM
```catkin build mloam```

### 3. Example
* Dataset 
  1. dataset 1: [Four LiDAR dataset for testing on pingshan, ShenZhen](http://gofile.me/4jm56/NNFbLc5cn)
  2. dataset 2: [Simulation Robot](http://gofile.me/4jm56/HzMDz6cvK)
  3. dataset 3: [Real Handheld Robot](http://gofile.me/4jm56/wJRrdgBwM)

* Run M-LOAM
  1. We provide a series of script to perform batch testing of M-LOAM with baselines
  2. Enter the script folder: ``roscd mloam/script/dataset_bash``
  3. Modify the shell files in ``test_main.sh`` with correct path, an example is shown as below:
    ```
    export data_path=$DATA_PATH/xxx/
    export rpg_path=$CATKIN_WS/src/localization/rpg_trajectory_evaluation
    export result_path=$rpg_path/results/xxx/
    mkdir -p $result_path/gf_pcd
    mkdir -p $result_path/traj
    mkdir -p $result_path/time
    mkdir -p $result_path/pose_graph
    mkdir -p $result_path/others
    bash test_main.sh
    ```
  4. ``bash test_debug.sh``
  5. Run mloam_loop: ```roslaunch mloam_loop mloam_loop_realvehicle.launch```

* Compare with baselines: M-LOAM-wo-ua, A-LOAM, F-LOAM, LEGO-LOAM
  1. Also modify the shell files in ``test_main.sh`` with correct commond, an example is shown as below:
  ```
  roslaunch mloam mloam_realvehicle_hercules.launch \
      run_mloam:=false \
      run_aloam:=true \
      data_path:=$data_path \
      data_source:=$data_source \
      delta_idx:=$delta_idx \
      start_idx:=$start_idx \
      end_idx:=$end_idx \
      output_path:=$result_path
  ```

### 5. Results
**red**: odometry; **green**: mapping; **blue**: gt

* Test in Simulation <br>
![](picture/simulation.png)

* Test in HKUST <br>
![](picture/hkust.png)

### 6. Additional Features (have not fixed)
* Future research
  1. Add a loop closure
  2. Object-centric SLAM
  3. Use more representative features
  4. Integrated with high-frequency sensors
  5. cross-domain, cross-modal dataset (simulator) for autonomous driving

* Threshold of UCT (scale = 1.0)
  1. 10m: 1.25
  2. 20m: 7.8
  3. 30m: 11.5
  4. 40m: 20
  5. 50m: 31.25

### 6. System pipeline
* Pipeline <br>
![](picture/mloam_pipeline.png)

### 7. Acknowledgements
Thanks for 

* LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and its advanced version: [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM);
* [LIO-MAPPING](https://github.com/hyye/lio-mapping) (Haoyang Ye, Yuying Chen, and Ming Liu. Tightly Coupled 3D Lidar Inertial Odometry and Mapping).
* VINS-MONO

<!-- ### 8. Compared with LEGO-LOAM
* Note: 0.2/0.4 (corner/surf resolution)
Algorithm                  | LEGO-LOAM  | M-LOAM 
---------------            | ----       | ---  
before ds                  | 18734/90578| 59494/78022
after ds                   | 11934/28110| 27954/14719
ds map time                | 6.63ms     | 12.08ms
input surf/corner num      | 3387/736   | 3785/2244
ds current scan time       | 0.5851ms   | 2.37ms
matching feature time      | 8.67ms     | 14.52ms
whole optimization time    | 60-100ms   | 100-200ms
save keyframes time        | 0.3913ms   | 0.000172ms
 -->
