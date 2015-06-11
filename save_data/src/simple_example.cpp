// TODO make one callback for rgb and depth, use only one lock


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <mutex>

bool save_images = false;
bool save_rgb = false;
bool save_depth = false;
std::mutex lock;
std::mutex rgb_lock;
std::mutex depth_lock;


void rgb_callback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("RGB CALLBACK ");
  rgb_lock.lock();
  if(save_rgb)
  {
    ROS_INFO("CALLBACK");
    try
    {
      cv::imshow("rgb", cv_bridge::toCvShare(msg, "bgr8")->image);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    save_rgb = false;
  }//if save images
  rgb_lock.unlock();
}//rgb_callback


void depth_callback(const sensor_msgs::ImageConstPtr& msg)
{
  depth_lock.lock();
  if(save_depth)
  {
    ROS_INFO("CALLBACK 2");
    try
    {
      cv::imshow("depth", cv_bridge::toCvShare(msg, msg->encoding)->image);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    save_depth = false;
  }//if save images
  depth_lock.unlock();
}//depth_callback

bool save(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ros::Duration(.1).sleep();
  rgb_lock.lock();
  depth_lock.lock();
  ROS_INFO("SERVICE");
  save_rgb = true;
  save_depth = true;
  depth_lock.unlock();
  rgb_lock.unlock();
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "saver");
  ros::NodeHandle nh;
  cv::namedWindow("rgb");
  cv::namedWindow("depth");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber rgb_sub = it.subscribe("/kinect2/rgb/image", 1, rgb_callback);
  image_transport::Subscriber depth_sub = it.subscribe("/kinect2/depth_highres/image", 1, depth_callback);

  ros::ServiceServer service =  nh.advertiseService("save", save);

  while(ros::ok()){
    ros::spinOnce();
   } 
  cv::destroyWindow("rgb");
  cv::destroyWindow("depth");
  ros::shutdown();
}

