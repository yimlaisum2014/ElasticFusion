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

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/simple_filter.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// A struct to hold the synchronized camera data 
// Struct to store stereo data
class StereoData
{
public:
  sensor_msgs::Image::ConstPtr image_l, image_r;
  sensor_msgs::CameraInfo::ConstPtr cam_info_l, cam_info_r;
  
  StereoData(const sensor_msgs::Image::ConstPtr &l_img, 
             const sensor_msgs::Image::ConstPtr &r_img, 
             const sensor_msgs::CameraInfo::ConstPtr &l_info, 
             const sensor_msgs::CameraInfo::ConstPtr &r_info) :
    image_l(l_img),
    image_r(r_img),
    cam_info_l(l_info),
    cam_info_r(r_info)
  {}
};

std::vector<StereoData> stereo_dataset_;

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function 
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    message_filters::SimpleFilter<M>::signalMessage(msg);
  }
};

// Callback for synchronized messages
void callback(const sensor_msgs::Image::ConstPtr &l_img, 
              const sensor_msgs::Image::ConstPtr &r_img, 
              const sensor_msgs::CameraInfo::ConstPtr &l_info,
              const sensor_msgs::CameraInfo::ConstPtr &r_info)
{
  StereoData sd(l_img, r_img, l_info, r_info);

  // Stereo dataset is class variable to store data
  stereo_dataset_.push_back(sd);
}
 
// Load bag
void loadBag(const std::string &filename)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Read);

  std::cout << "opening bag" << std::endl;
  
  std::string image_ns_ = "cam";

  std::string l_cam = image_ns_ + "/left";
  std::string r_cam = image_ns_ + "/right";
  std::string l_cam_image = l_cam + "/image_raw";
  std::string r_cam_image = r_cam + "/image_raw";
  std::string l_cam_info = l_cam + "/camera_info";
  std::string r_cam_info = r_cam + "/camera_info";
  
  // Image topics to load
  std::vector<std::string> topics;
  topics.push_back(l_cam_image);
  topics.push_back(r_cam_image);
  topics.push_back(l_cam_info);
  topics.push_back(r_cam_info);
  
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  
  // Set up fake subscribers to capture images
  BagSubscriber<sensor_msgs::Image> l_img_sub, r_img_sub;
  BagSubscriber<sensor_msgs::CameraInfo> l_info_sub, r_info_sub;
  
  // Use time synchronizer to make sure we get properly synchronized images
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> sync(l_img_sub, r_img_sub, l_info_sub, r_info_sub, 25);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
  
  // Load all messages into our stereo dataset
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if (m.getTopic() == l_cam_image || ("/" + m.getTopic() == l_cam_image))
    {
      sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
      if (l_img != NULL)
        l_img_sub.newMessage(l_img);
    }
    
    if (m.getTopic() == r_cam_image || ("/" + m.getTopic() == r_cam_image))
    {
      sensor_msgs::Image::ConstPtr r_img = m.instantiate<sensor_msgs::Image>();
      if (r_img != NULL)
        r_img_sub.newMessage(r_img);
    }
    
    if (m.getTopic() == l_cam_info || ("/" + m.getTopic() == l_cam_info))
    {
      sensor_msgs::CameraInfo::ConstPtr l_info = m.instantiate<sensor_msgs::CameraInfo>();
      if (l_info != NULL)
        l_info_sub.newMessage(l_info);
    }
    
    if (m.getTopic() == r_cam_info || ("/" + m.getTopic() == r_cam_info))
    {
      sensor_msgs::CameraInfo::ConstPtr r_info = m.instantiate<sensor_msgs::CameraInfo>();
      if (r_info != NULL)
        r_info_sub.newMessage(r_info);
    }
  }
  bag.close();
}

ROSBagReader::ROSBagReader(std::string file, bool flipColors)
 : LogReader(file, flipColors)
{
    std::cout << "constructing ROSBagReader" << std::endl;
    assert(pangolin::FileExists(file.c_str()));
    
    loadBag(file);
    exit(0);

    //logFile = new lcm::LogFile(file, "r");
    //fp = logFile->getFilePtr();

    currentFrame = 0;

    // now is when I read over the entire ROS Bag


    numFrames = 10000;

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

}

void ROSBagReader::getBack()
{
    assert(filePointers.size() > 0);

    fseek(fp, filePointers.top(), SEEK_SET);

    filePointers.pop();

    getCore();
}

void ROSBagReader::getNext()
{
    filePointers.push(ftell(fp));

    getCore();
}

void ROSBagReader::getCore()
{
    std::string channel = "";
    //const lcm::LogEvent* event = 0;

    // while (channel != "OPENNI_FRAME") {
    //     event = logFile->readNextEvent();
    //     channel = event->channel;
    //     //std::cout << "read event on channel: " << channel << std::endl;
    // }

    // bot_core::images_t message;
    // message.decode(event->data, 0, event->datalen);
    // timestamp = message.utime;

    // std::cout << "timestamp: " << timestamp << std::endl;
    // std::cout << "frame: " << currentFrame << std::endl;

    // bot_core::image_t colorImage = message.images[0];
    // bot_core::image_t depthImage = message.images[1];


    // bool isZlibCompressed = false;

    // if (depthImage.pixelformat == bot_core::image_t::PIXEL_FORMAT_INVALID)
    // {
    //   isZlibCompressed = true;
    // }

    // depthSize = depthImage.size;
    // imageSize = colorImage.size;


/*
    auto tmp = fread(&timestamp,sizeof(int64_t),1,fp);
    assert(tmp);
    tmp = fread(&depthSize,sizeof(int32_t),1,fp);
    assert(tmp);
    tmp = fread(&imageSize,sizeof(int32_t),1,fp);
    assert(tmp);
    tmp = fread(depthReadBuffer,depthSize,1,fp);
    assert(tmp);
    if(imageSize > 0)
    {
        tmp = fread(imageReadBuffer,imageSize,1,fp);
        assert(tmp);
    }
*/

    // if(depthSize == numPixels * 2)
    // {
    //    // printf("copying depth image\n");
    //     memcpy(&decompressionBufferDepth[0], depthImage.data.data(), numPixels * 2);
    // }
    // else
    // {
    //   // printf("uncompress depth image\n");
    //     unsigned long decompLength = numPixels * 2;
    //     uncompress(&decompressionBufferDepth[0], (unsigned long *)&decompLength, (const Bytef *)depthImage.data.data(), depthSize);
    // }

    // if(imageSize == numPixels * 3)
    // {
    //   //  printf("copy color image\n");
    //     memcpy(&decompressionBufferImage[0], colorImage.data.data(), numPixels * 3);
    // }
    // else if(imageSize > 0)
    // {
    //   //  printf("jpeg read color image\n");

    //     jpeg.readData(colorImage.data.data(), imageSize, (unsigned char *)&decompressionBufferImage[0]);
    // }
    // else
    // {
    //     memset(&decompressionBufferImage[0], 0, numPixels * 3);
    // }

    // depth = (unsigned short *)decompressionBufferDepth;
    // rgb = (unsigned char *)&decompressionBufferImage[0];

    // if(flipColors)
    // {
    //     for(int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3)
    //     {
    //         std::swap(rgb[i + 0], rgb[i + 2]);
    //     }
    // }

    currentFrame++;
}

void ROSBagReader::fastForward(int frame)
{
  printf("ROSBagReader::fastForward not implemented\n");
/*
    while(currentFrame < frame && hasMore())
    {
        filePointers.push(ftell(fp));
        auto tmp = fread(&timestamp,sizeof(int64_t),1,fp);
        assert(tmp);
        tmp = fread(&depthSize,sizeof(int32_t),1,fp);
        assert(tmp);
        tmp = fread(&imageSize,sizeof(int32_t),1,fp);
        assert(tmp);
        tmp = fread(depthReadBuffer,depthSize,1,fp);
        assert(tmp);
        if(imageSize > 0)
        {
            tmp = fread(imageReadBuffer,imageSize,1,fp);
            assert(tmp);
        }
        currentFrame++;
    }
*/
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
    if (filePointers.size() != 0)
    {
        std::stack<int> empty;
        std::swap(empty, filePointers);
    }
/*
    fclose(fp);
    fp = fopen(file.c_str(), "rb");
    auto tmp = fread(&numFrames,sizeof(int32_t),1,fp);
    assert(tmp);
    currentFrame = 0;
*/

    printf("ROSBagReader::rewind\n");
    /*
    delete logFile;
    logFile = new lcm::LogFile(file, "r");
    fp = logFile->getFilePtr();
    */

    currentFrame = 0;
}

bool ROSBagReader::rewound()
{
    return filePointers.size() == 0;
}

const std::string ROSBagReader::getFile()
{
    return file;
}

void ROSBagReader::setAuto(bool value)
{

}