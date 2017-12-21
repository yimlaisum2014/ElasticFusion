#!/bin/bash
use_ros

#Installing Pangolin
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ../ -DAVFORMAT_INCLUDE_DIR="" -DCPP11_NO_BOOST=ON -DBUILD_PANGOLIN_LIBREALSENSE=OFF
make -j8
cd ../..

#Up to date OpenNI2
git clone https://github.com/occipital/OpenNI2.git
cd OpenNI2
make -j8

#Actually build ElasticFusion
cd ../Core
mkdir build
cd build
cmake ../src
make -j8
cd ../../GPUTest
mkdir build
cd build
cmake ../src
make -j8
cd ../../GUI
mkdir build
cd build
cmake ../src
make -j8
