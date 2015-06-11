#ifndef __SERVER_SAVER_H__
#define __SERVER_SAVER_H__




#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>


#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>


#include <cv_bridge/cv_bridge.h>


#include <image_transport/image_transport.h>


#include <kinect2_definitions.h>

#include <string>
#include <sstream>






class Receiver
{
private:
  std::mutex lock;
  std::string topic_color, topic_depth;
  size_t queue_size;
  
  int counter;
  std::string savePath;

  cv::Mat color, depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;

  ros::NodeHandle nh;
//  image_transport::ImageTransport it;
 // image_transport::Subscriber sub;

 

public:
  Receiver(const std::string &topicColor, const std::string &topicDepth);

  void run();


  void image_callback(const sensor_msgs::ImageConstPtr& msg);


};
#endif
