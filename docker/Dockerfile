FROM ros:melodic-perception

ENV CERES_VERSION="1.12.0"
ENV PCL_VERSION="1.8.0"
ENV CATKIN_WS=/usr/app/catkin_ws
ENV DATA_PATH=/usr/app/dataset
ENV LIBRARY=/usr/app/library

RUN mkdir -p $CATKIN_WS/src/localization
RUN mkdir $LIBRARY

# setup processors number used to compile library
RUN if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; else export USE_PROC=$(($(nproc)/2)) ; fi && \
    # Install dependencies
      apt-get update && apt-get install -y \
      locate \
      cmake \
      wget \
      vim \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      python-catkin-tools \
      autoconf \
      automake \
      libtool \
      python-pip \
      python-tk \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-tf && \
      ros-${ROS_DISTRO}-tf-conversions && 
      ros-$(ROS_DISTRO)-joint-state-publisher && \
      ros-$(ROS_DISTRO)-robot-state-publisher && \
    pip install numpy matplotlib colorama ruamel.yaml && \
    # rm -rf /var/lib/apt/lists/* && \
    # Build and install Ceres
    cd $LIBRARY && \
    git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && \
    git checkout tags/${CERES_VERSION} && \
    mkdir build && cd build && \
    cmake .. && \
    make -j${USE_PROC} install && \
    cd ../.. && \
    rm -rf ./ceres-solver && \
    # Build and install pcl
    cd $LIBRARY && \
    git clone https://github.com/PointCloudLibrary/pcl.git && \
    cd pcl && \
    git checkout tags/pcl-${PCL_VERSION} && \
    mkdir build && cd build && \
    cmake .. && \
    make -j${USE_PROC} install && \
    cd ../.. && \
    rm -rf ./pcl && \
    # Build and install gtsam
    cd $LIBRARY && \
    git clone https://github.com/borglab/gtsam && \
    cd gtsam && \
    mkdir build && cd build && \
    cmake .. && \
    make -j${USE_PROC} install && \
    cd ../../ && \
    rm -rf ./gtsam && \
    # Build and install g2o
    # cd $LIBRARY && \  
    # git clone https://github.com/RainerKuemmerle/g2o && \
    # cd gtsam && \
    # mkdir build && cd build && \
    # cmake .. && \
    # make -j${USE_PROC} install && \
    # cd ../../ && \
    # rm -rf ./gtsam && \
    ### Install GFLAGS
    cd $LIBRARY && \
    git clone https://github.com/gflags/gflags && \
    cd gflags && \
    cmake . && make && make install && \
    cd $LIBRARY && \
    rm -rf gflags && \
    ### Install gtest
    cd $LIBRARY && \
    git clone https://github.com/google/googletest && \
    cd googletest && \
    cmake . && make && make install && \
    cd $LIBRARY && \
    rm -rf googletest 
    ### Install glog
    # git clone https://github.com/gflags/gflags && \
    # cd gflags && \
    # ./configure && \
    # make && make install \
    # Setup catkin workspace

#################################### code can be copied or cloned from git
# Copy M-LOAM
# COPY ../../M-LOAM/ $CATKIN_WS/src/localization/M-LOAM/
# use the following line if you only have this dockerfile
# RUN git clone https://github.com/HKUST-Aerial-Robotics/A-LOAM.git

# Copy A-LOAM, LEGO-LOAM, rpg
# COPY ../../LeGO-LOAM/ $CATKIN_WS/src/localization/LeGO-LOAM/
# COPY ../../aloam_velodyne/ $CATKIN_WS/src/localization/aloam_velodyne/

# Build M-LOAM, A-LOAM, LEGO-LOAM
# WORKDIR $CATKIN_WS
# ENV TERM xterm
# ENV PYTHONIOENCODING UTF-8
# RUN catkin config \
#       --extend /opt/ros/$ROS_DISTRO \
#       --cmake-args \
#         -DCMAKE_BUILD_TYPE=Release && \
#     catkin build && \
#     sed -i '/exec "$@"/i \
#             source "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh
