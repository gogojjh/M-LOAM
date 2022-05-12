# M-LOAM
## Robust Odometry and Mapping for Multi-LiDAR Systems with Online Extrinsic Calibration
M-LOAM is a robust system for multi-LiDAR extrinsic calibration, real-time odometry, and mapping. Without manual intervention, our system can start with several extrinsic-uncalibrated LiDARs, automatically calibrate their extrinsics, and provide accurate poses as well as a globally consistent map.

**Authors:** 
[Jianhao Jiao](http://gogojjh.github.io), 
[Haoyang Ye](https://github.com/hyye),
[Yilong Zhu](https://scholar.google.com/citations?user=x8n6v2oAAAAJ&hl=zh-CN),
[Linxin Jiang](https://github.com/jianglingxin),
[Ming Liu](https://scholar.google.com/citations?user=CdV5LfQAAAAJ&hl=zh-CN)
from [RAM-LAB](https://www.ram-lab.com), [HKUST](http://www.ust.hk)

**Project website:** https://ram-lab.com/file/site/m-loam

**Videos:**

<a href="https://www.youtube.com/watch?v=VqaIb3GaCmE" target="_blank"><img src="https://img.youtube.com/vi/VqaIb3GaCmE/0.jpg" 
alt="mloam" width="320" height="210" border="10" /></a>

(Video link for mainland China friends: <a href="https://www.bilibili.com/video/BV1ur4y1K7FR/">Video</a>)

**Related Papers in Solving Different Subproblems for Multi-LiDAR Systems**
* **Robust Odometry and Mapping for Multi-LiDAR Systems with Online Extrinsic Calibration**, **Jianhao Jiao**, Haoyang Ye, Yilong Zhu, Ming Liu, IEEE Transactions on Robotics (*T-RO*), 2021. [pdf](https://arxiv.org/pdf/2010.14294.pdf)
  - Tackle the extrinsic calibration, multi-LiDAR fusion, pose drift, and mapping uncertainty.

* **Greedy-Based Feature Selection for Efficient LiDAR SLAM**, **Jianhao Jiao**, Yilong Zhu, Haoyang Ye, Huaiyang Huang, Peng Yun, Linxin Jiang, Lujia Wang, Ming Liu, 
International Conference on Robotics and Automation (*ICRA*) 2021 , Xi An, China. [pdf](https://www.ram-lab.com/papers/2021/jiao2021greedy.pdf)
  - Tackle the algorithm latency issue.

* **MLOD: Awareness of Extrinsic Perturbation in Multi-LiDAR 3D Object Detection for Autonomous Driving**, **Jianhao Jiao***, Peng Yun*, Lei Tai, Ming Liu, IEEE/RSJ International Conference on Intelligent Robots and Systems (*IROS*) 2020. [pdf](https://arxiv.org/abs/2010.11702.pdf)
  - Tackle the multi-LiDAR-based 3D object detection against the hardware failure (injected large extrinsic perturbation).

*If you use M-LOAM for your academic research, please cite one of our paper.* [bib](https://github.com/gogojjh/M-LOAM/blob/mloam_gf/docs/support_files/paper_bib.txt)

<!-- ----------------------------------------------------------- -->
### 1. Prerequisites
1.1 **Ubuntu** and **ROS**

Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

1.2. **Ceres Solver && Eigen3 && GLOG**

```
 ./setup/install_eigen3_ceres.sh
```

1.3. **OpenMP**
```
  sudo apt install libomp-dev
```

1.4. **PCL-1.8**

### 2. Build M-LOAM on ROS
```
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  git clone https://github.com/gogojjh/M-LOAM.git
  catkin build mloam
  source ~/catkin_ws/devel/setup.bash
```

<!-- ----------------------------------------------------------- -->
### 3. Example
* [Datasets](http://gofile.me/4jm56/2iYvOr73R) collected with different platforms:
  1. Simulation Robot (SR)
  2. Real Handheld Device (RHD)
  3. Real Vechile (RV)
  4. Oxford RoboCar (OR)

* Run M-LOAM and baseline methods
  1. We provide a script to perform batch testing of M-LOAM with baseline methods
  2. Enter the script folder: ``roscd mloam/script/``
  3. Modify the python script: ``run_mloam.py`` for specific platforms with correct path
  4. Modify the shell files for methods in ``xx_main.sh``
  5. Run the python script: 
  * ``python2 run_mloam.py -program=single_test -sequence=SR -start_idx=0 -end_idx=4`` 
    * You will broadcast the SR01.bag, SR02.bag, SR03.bag, SR04.bag, SR05.bag respectively to test the M-LOAM system.
  * ``python2 run_mloam.py -program=single_test -sequence=RHD -start_idx=0 -end_idx=2`` 
    * You will broadcast the RHD02lab.bag, RHD03garden.bag, RHD04building.bag respectively to test the M-LOAM system.
  * ``python2 run_mloam.py -program=single_test -sequence=RHD -start_idx=1 -end_idx=1`` 
    * You will broadcast the RV01.bag to test the M-LOAM system.

<!-- ----------------------------------------------------------- -->
### 4. System pipeline
This could help you to understand the pipeline of M-LOAM (loop closure part is not finished). 
<img src="docs/picture/mloam_pipeline.png" style="zoom:67%;" />

And you can also refer to [M-LOAM's pipeline](docs/mloam-pipeline-chinese.pdf) for a more detailed diagram and code review.

<!-- ----------------------------------------------------------- -->
### 5. Acknowledgements
Thanks for these great works from which we learned to develop M-LOAM

* LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and its advanced version: [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM);
* [LEGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)
* [LIO-MAPPING](https://github.com/hyye/lio-mapping)
* [VINS-MONO](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
* [Lidar Perception Library](https://github.com/LidarPerception/common_lib)

Thanks for Ming Cao for providing a clear diagram and code review of M-LOAM

<!-- ----------------------------------------------------------- -->
### 6. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

For any technical issues, please contact Dr. Jianhao Jiao <jiaojh1994@gmail.com>. For commercial inquiries, please contact Prof.Ming Liu <eelium@ust.hk>.
