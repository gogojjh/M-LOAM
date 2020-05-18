# M-LOAM
### Multi-LiDAR Odometry and Mapping

M-LOAM is xxx

<!-- A-LOAM is an Advanced implementation of LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), which uses Eigen and Ceres Solver to simplify code structure. This code is modified from LOAM and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED). This code is clean and simple without complicated mathematical derivation and redundant operations. It is a good learning material for SLAM beginners. -->

<!-- <img src="https://github.com/HKUST-Aerial-Robotics/A-LOAM/blob/devel/picture/kitti.png" width = 55% height = 55%/> -->

**Modifier:** [Jianhao Jiao](http://gogojjh.github.io), [Haoyang Ye](https://github.com/xxx)


### 1. Prerequisites
#### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

#### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

#### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).
> NOTE: Starting with PCL-1.7 you need to define PCL_NO_PRECOMPILE before you include any PCL headers to include the templated algorithms as well.

#### 1.4. **OpenMP**
```sudo apt install libomp-dev```

#### 1.5. **Eigen3**
```sudo apt install libeigen-dev```
> NOTE: to prevent Eigen error: https://eigen.tuxfamily.org/dox/group__TopicUnalignedArrayAssert.html

### 2. Build M-LOAM
```catkin build mloam```

### 3. Example
```
    rosbag play xxx.bag -l 
    roslaunch mloam mloam_handheld.launch
    rostopic echo /extrinsics/odoms[1]/pose
```

### 4. Compare with A-LOAM
> mloam_handheld.launch set run_aloam:=true
```roslaunch mloam mloam_handheld.launch```

### 5. Results
**red**: odometry; **green**: mapping; **blue**: gt

* Test in Simulation
![](picture/simulation.png)

* Test in HKUST
![](picture/hkust.png)
    
### 6. Additional Features (have not fixed)

### 7. Acknowledgements
Thanks for LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).
