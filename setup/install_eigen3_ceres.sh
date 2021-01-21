#!/bin/bash

set -o errexit
set -o verbose

sudo apt-get install cmake -y
sudo apt-get install libgoogle-glog-dev -y
sudo apt-get install libatlas-base-dev -y
sudo apt-get install libeigen3-dev -y
sudo apt-get install libsuitesparse-dev -y
sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
sudo apt-get update
sudo apt-get install libsuitesparse-dev -y
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
mkdir build && cd build
cmake ..
make -j4
sudo make install

exit 0