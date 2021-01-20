#!/bin/bash

set -o errexit
set -o verbose

git clone https://github.com/ethz-asl/libnabo.git
cd libnabo
git checkout tags/1.0.7
mkdir build && cd build
cmake ..
make -j4
sudo make install

exit 0
