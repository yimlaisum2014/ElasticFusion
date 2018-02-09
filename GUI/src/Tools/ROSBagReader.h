/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 *
 * The use of the code within this file and all code within files that
 * make up the software that is ElasticFusion is permitted for
 * non-commercial purposes only.  The full terms and conditions that
 * apply to the code within this file are detailed within the LICENSE.txt
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/>
 * unless explicitly stated.  By downloading this file you agree to
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#ifndef ROSBAGREADER_H_
#define ROSBAGREADER_H_

#include <Utils/Resolution.h>
#include <Utils/Stopwatch.h>
#include <pangolin/utils/file_utils.h>

#include "LogReader.h"

#include <cassert>
#include <zlib.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <stack>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

struct ROSLogData{
    std::string ros_bag_filename;
    std::string image_depth_topic;
    std::string image_rgb_topic;
    std::string cam_info_topic;
};

// Helper functions to enable loading of calibration params before creating a LogReader instance
bool isRosBag(std::string const& value);
void rosGetParams(const ROSLogData & log_data , int& pixels_width, int& pixels_height, double& fx, double& fy, double& cx, double& cy);



// Struct to store rgbd data
class ROSRgbdData
{
public:
  std::vector<sensor_msgs::Image::ConstPtr> images_rgb;
  std::vector<sensor_msgs::Image::ConstPtr> images_d;
  sensor_msgs::CameraInfo::ConstPtr cam_info;       // Note: assumes depth-rgb has already been registered 
};

class ROSBagReader : public LogReader
{
    public:
        ROSBagReader(const ROSLogData & log_data, bool flipColors);

        virtual ~ROSBagReader();

        void getNext();

        void getBack();

        int getNumFrames();

        bool hasMore();

        bool rewound();

        void rewind();

        void fastForward(int frame);

        const std::string getFile();

        void setAuto(bool value);

        std::stack<int> filePointers;

    private:
        void getCore();
        ROSRgbdData log_rgbd_data;
};

#endif /* ROSBAGREADER_H_ */
