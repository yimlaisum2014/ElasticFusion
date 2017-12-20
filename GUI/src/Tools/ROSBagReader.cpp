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

#include "ROSBagReader.h"

// Load bag
void loadBag(const std::string &filename, ROSRgbdData& log_rgbd_data)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Read);

  std::cout << "opening bag" << std::endl;
  std::cout << "rgbd dataset size " << log_rgbd_data.images_d.size() << std::endl;
  
  std::string image_d_topic = "/camera_1112170110/depth_registered/sw_registered/image_rect";
  std::string image_rgb_topic = "/camera_1112170110/rgb/image_rect_color";
  std::string cam_info_d_topic = "/camera_1112170110/depth_registered/sw_registered/camera_info";
  std::string cam_info_rgb_topic = "/camera_1112170110/rgb/camera_info";
  
  // Image topics to load
  std::vector<std::string> topics;
  topics.push_back(image_d_topic);
  topics.push_back(image_rgb_topic);
  topics.push_back(cam_info_d_topic);
  topics.push_back(cam_info_rgb_topic);
  
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  
  // Load all messages into our stereo dataset
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if (m.getTopic() == image_d_topic || ("/" + m.getTopic() == image_d_topic))
    {
      sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
      if (l_img != NULL) {
        log_rgbd_data.images_d.push_back(l_img);
      }
    }
    
    if (m.getTopic() == image_rgb_topic || ("/" + m.getTopic() == image_rgb_topic))
    {
      sensor_msgs::Image::ConstPtr r_img = m.instantiate<sensor_msgs::Image>();
      if (r_img != NULL) {
        log_rgbd_data.images_rgb.push_back(r_img);
      }
    }
    
    if (m.getTopic() == cam_info_d_topic || ("/" + m.getTopic() == cam_info_d_topic))
    {
      sensor_msgs::CameraInfo::ConstPtr l_info = m.instantiate<sensor_msgs::CameraInfo>();
      if (l_info != NULL) {
        log_rgbd_data.cam_info_d = l_info;
      }
    }
    
    if (m.getTopic() == cam_info_rgb_topic || ("/" + m.getTopic() == cam_info_rgb_topic))
    {
      sensor_msgs::CameraInfo::ConstPtr r_info = m.instantiate<sensor_msgs::CameraInfo>();
      if (r_info != NULL) {
        log_rgbd_data.cam_info_d = r_info;
      }
    }
  }
  bag.close();
  std::cout << "closing bag" << std::endl;
  std::cout << "rgb data size " << log_rgbd_data.images_rgb.size() << std::endl;
  std::cout << "d data size " << log_rgbd_data.images_d.size() << std::endl;
}


ROSBagReader::ROSBagReader(std::string file, bool flipColors)
 : LogReader(file, flipColors)
{
    std::cout << "constructing ROSBagReader" << std::endl;
    assert(pangolin::FileExists(file.c_str()));
    
    // Load in all of the ros bag into an ROSRgbdData strcut
    loadBag(file, log_rgbd_data);

    fp = fopen(file.c_str(), "rb");

    currentFrame = 0;

    // if (log_rgbd_data.images_rgb.size() != log_rgbd_data.images_d.size()) {
    //   std::cout << "Need to implement time sync!" << std::endl;
    //   exit(0);
    // }
    numFrames = log_rgbd_data.images_rgb.size();

    depthReadBuffer = new unsigned char[numPixels * 2];
    imageReadBuffer = new unsigned char[numPixels * 3];
    decompressionBufferDepth = new Bytef[numPixels * 2];
    decompressionBufferImage = new Bytef[numPixels * 3];
}

ROSBagReader::~ROSBagReader()
{
    delete [] depthReadBuffer;
    delete [] imageReadBuffer;
    delete [] decompressionBufferDepth;
    delete [] decompressionBufferImage;

    fclose(fp);
}

void ROSBagReader::getBack()
{
    getCore();
}

void ROSBagReader::getNext()
{
    std::cout << "in getNext" << std::endl;
    getCore();
}

void ROSBagReader::getCore()
{
    std::cout << "in getCore" << std::endl;

    timestamp = log_rgbd_data.images_rgb.at(currentFrame)->header.stamp.toSec();

    std::cout << log_rgbd_data.images_rgb.at(currentFrame)->header.stamp << "was rgb timestamp" << std::endl;
    std::cout << log_rgbd_data.images_d.at(currentFrame)->header.stamp << "was d timestamp" << std::endl;

    std::cout << log_rgbd_data.images_rgb.at(currentFrame+1)->header.stamp << "was rgb timestamp" << std::endl;
    std::cout << log_rgbd_data.images_d.at(currentFrame+1)->header.stamp << "was d timestamp" << std::endl;

    std::cout << log_rgbd_data.images_rgb.at(0)->encoding << " is rgb encoding" << std::endl;
    std::cout << "height " << log_rgbd_data.images_rgb.at(0)->height << std::endl;
    std::cout << "width " << log_rgbd_data.images_rgb.at(0)->width << std::endl;
    std::cout << log_rgbd_data.images_d.at(0)->encoding << " is d encoding" << std::endl;
 

    depthSize = log_rgbd_data.images_d.at(0)->step * log_rgbd_data.images_d.at(0)->height;
    imageSize = log_rgbd_data.images_rgb.at(0)->step * log_rgbd_data.images_rgb.at(0)->height;
    std::cout << "depthSize " << depthSize << std::endl;
    std::cout << "imageSize " << imageSize << std::endl; 

    // Depth
    if(depthSize == numPixels * 2)
    {
        printf("copying depth image\n");
        memcpy(&decompressionBufferDepth[0], &(log_rgbd_data.images_d.at(currentFrame)->data[0]), numPixels * 2);
    } else if (depthSize == numPixels * 4) {
        printf("Need to convert from 32FC1 to 16UC1\n");
        cv::Mat cv_depth = cv::Mat(log_rgbd_data.images_d.at(0)->height, log_rgbd_data.images_d.at(0)->width, CV_32FC1); 
        for (size_t i = 0; i < log_rgbd_data.images_d.at(currentFrame)->height; i++) {
            for (size_t j = 0; j < log_rgbd_data.images_d.at(currentFrame)->width; j++) {
                cv_depth.at<float>(i,j) = *( (float*) &(log_rgbd_data.images_d.at(currentFrame)->data[0]) + (i*log_rgbd_data.images_d.at(currentFrame)->width + j) );
            }
        }
        std::cout << cv_depth.size() << std::endl;
        cv::Mat cv_depth_out = cv::Mat(log_rgbd_data.images_d.at(0)->height, log_rgbd_data.images_d.at(0)->width, CV_16UC1);
        cv_depth.convertTo(cv_depth_out, CV_16UC1, 1000);
        std::cout << cv_depth_out.size() << std::endl; 
        memcpy(&decompressionBufferDepth[0], cv_depth_out.ptr(), numPixels * 2);
        printf("finished the conversion\n");
    }
    else
    {
      printf("haven't implemented decompressing depth image\n");
      exit(0);
    }

    printf("finished managing depth image\n");


    // RGB
    if(imageSize == numPixels * 3)
    {
        memcpy(&decompressionBufferImage[0], &(log_rgbd_data.images_rgb.at(currentFrame)->data[0]), numPixels * 3);
    } else {
        printf("haven't implemented decompressing depth image\n");
        exit(0);
    }

    rgb = (unsigned char *)&decompressionBufferImage[0];
    depth = (unsigned short *)&decompressionBufferDepth[0];

    if(flipColors)
    {
        for(int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }

    currentFrame++;
}

void ROSBagReader::fastForward(int frame)
{
  printf("ROSBagReader::fastForward not implemented\n");
}

int ROSBagReader::getNumFrames()
{
    return numFrames;
}

bool ROSBagReader::hasMore()
{
    return currentFrame + 1 < numFrames;
}


void ROSBagReader::rewind()
{
    printf("ROSBagReader::rewind\n");
    currentFrame = 0;
}

bool ROSBagReader::rewound()
{
    return (currentFrame == 0);
}

const std::string ROSBagReader::getFile()
{
    return file;
}

void ROSBagReader::setAuto(bool value)
{

}