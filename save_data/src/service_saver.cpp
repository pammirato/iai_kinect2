// TODO make one callback for rgb, depth and raw_depth, use only one lock


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <mutex>
#include <sys/stat.h>

#include <kinect2_definitions.h>

namespace patch
{
    template < typename T > std::string to_string( const T& n ) 
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}






const std::string base_save_path = "/home/ammirato/Documents/Kinect/Data/K2/SimpleGridMotion/First/";
const std::string rgb_save_path = base_save_path +"rgb/";
const std::string depth_save_path =base_save_path +"depth/";
const std::string raw_depth_save_path =base_save_path + "raw_depth/";
const std::string rgb_save_name =  "rgb";
const std::string depth_save_name =  "depth";
const std::string raw_depth_save_name= "raw_depth";

const std::string image_extension = ".png";



int counter = -1;
bool save_images = false;
bool save_rgb = false;
bool save_depth = false;
bool save_raw_depth = false;
std::mutex lock;
std::mutex rgb_lock;
std::mutex depth_lock;
std::mutex raw_depth_lock;
cv::Mat rgb;
cv::Mat depth;
cv::Mat raw_depth;

bool  saved;


void rgb_callback(const sensor_msgs::ImageConstPtr& msg)
{
  rgb_lock.lock();
  if(save_rgb)
  {
    ROS_INFO("RGB SAVE CALLBACK");
    try
    {
      //convert message too opencv mat
      rgb = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::imshow("rgb",rgb );
      cv::resizeWindow("rgb",216, 384);
      
      cv::imwrite(rgb_save_path + rgb_save_name + patch::to_string(counter) + image_extension, rgb);
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
  ROS_INFO("DP");
  depth_lock.lock();
  if(save_depth)
  {
    ROS_INFO("DEPTH SAVE CALLBACK");
    try
    {
      depth = cv_bridge::toCvShare(msg, msg->encoding)->image;
      cv::imshow("depth",depth );
      cv::resizeWindow("depth",216, 384);
      
      saved = cv::imwrite( depth_save_path + depth_save_name + patch::to_string(counter) + image_extension, depth);
      cv::waitKey(30);
      ROS_INFO("saved: %d",saved);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    save_depth = false;
  }//if save images
  depth_lock.unlock();
}//depth_callback





void raw_depth_callback(const sensor_msgs::ImageConstPtr& msg)
{
  raw_depth_lock.lock();
  if(save_raw_depth)
  {
    ROS_INFO("RAW_DEPTH SAVE CALLBACK");
    try
    {
      raw_depth = cv_bridge::toCvShare(msg, msg->encoding)->image;
      //cv::imshow("raw_depth",raw_depth );
      //cv::resizeWindow("raw_depth",216, 384);
      
      cv::imwrite(raw_depth_save_path + raw_depth_save_name + patch::to_string(counter) + image_extension, raw_depth);

      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    save_raw_depth = false;
  }//if save images
  raw_depth_lock.unlock();
}//raw_depth_callback













bool save(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  counter++;
  ros::Duration(.1).sleep();
  rgb_lock.lock();
  depth_lock.lock();
  raw_depth_lock.lock();
  ROS_INFO("SERVICE");
  save_rgb = true;
  save_depth = true;
  save_raw_depth = true;
  raw_depth_lock.unlock();
  depth_lock.unlock();
  rgb_lock.unlock();
  return true;
}


int main(int argc, char **argv)
{

  mkdir(rgb_save_path.c_str(), 0777);
  mkdir(depth_save_path.c_str(), 0777);
  mkdir(raw_depth_save_path.c_str(), 0777);

  ros::init(argc, argv, "saver");
  ros::NodeHandle nh;

  std::string ns = K2_DEFAULT_NS;

  cv::namedWindow("rgb");
  cv::namedWindow("depth");
  cv::startWindowThread();


  image_transport::ImageTransport it(nh);

  image_transport::Subscriber rgb_sub = it.subscribe(ns + K2_TOPIC_IMAGE_COLOR + K2_TOPIC_RAW, 1, rgb_callback);
  image_transport::Subscriber depth_sub = it.subscribe(ns + K2_TOPIC_HIRES_DEPTH + K2_TOPIC_RAW, 1, depth_callback);
  image_transport::Subscriber raw_depth_sub = it.subscribe(ns + K2_TOPIC_IMAGE_DEPTH + K2_TOPIC_RAW, 1, raw_depth_callback);

  ros::ServiceServer service =  nh.advertiseService("save_images", save);

  while(ros::ok()){
    ros::spinOnce();
   } 
  cv::destroyWindow("rgb");
  cv::destroyWindow("depth");
  ros::shutdown();
}

