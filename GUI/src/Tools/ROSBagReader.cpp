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
  std::cout << "rgbd dataset size" << log_rgbd_data.images_d.size() << std::endl;
  
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
  std::cout << "rgbd dataset size" << log_rgbd_data.images_d.size() << std::endl;
}

ROSBagReader::ROSBagReader(std::string file, bool flipColors)
 : LogReader(file, flipColors)
{
    std::cout << "constructing ROSBagReader" << std::endl;
    assert(pangolin::FileExists(file.c_str()));
    
    loadBag(file, log_rgbd_data);
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