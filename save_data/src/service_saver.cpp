// TODO make one callback for rgb, depth and raw_depth, use only one lock


#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <mutex>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

//#include <ros/spinner.h>
#include <ros/callback_queue.h>


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>





#include <kinect2_definitions.h>

/*namespace std
{
    template < typename T > std::string to_string( const T& n ) 
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}
*/

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::Image> ExactSyncPolicy;



const std::string base_save_path_1 = "/home/ammirato/Documents/Kinect/Data/";
const std::string base_save_path_2 = "/SimpleGridMotion/Test/";
std::string rgb_save_path = "rgb/";
std::string depth_save_path = "depth/";
std::string raw_depth_save_path = + "raw_depth/";
const std::string rgb_save_name =  "rgb/rgb";
const std::string depth_save_name =  "depth/depth";
const std::string raw_depth_save_name= "raw_depth/raw_depth";

const std::string image_extension = ".png";

std::string base_name = "kinect2";

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

std::vector<int> compression_params;
    

image_transport::SubscriberFilter *rgb_filter_sub, *depth_filter_sub, *raw_depth_filter_sub; 
message_filters::Synchronizer<ExactSyncPolicy> *syncExact;

std::string timestamp_sec;
std::string timestamp_nsec;


bool  saved;


void images_callback(const sensor_msgs::Image::ConstPtr rgb_msg, const sensor_msgs::Image::ConstPtr depth_msg, const sensor_msgs::Image::ConstPtr raw_depth_msg)
{
  ROS_INFO("images -callback");
  lock.lock();
  ROS_INFO("images -callback got lock");
  if(save_images)
  {
    ROS_INFO("SAVE CALLBACK");
    try
    {
      //convert message too opencv mat
      rgb = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding)->image;
      cv::imshow("rgb",rgb );
      cv::resizeWindow("rgb",432, 768);
      std::string timestamp_sec =std::to_string(rgb_msg->header.stamp.sec); 
      std::string timestamp_nsec=std::to_string(rgb_msg->header.stamp.nsec); 
      cv::imwrite(rgb_save_path + timestamp_sec + "_" + timestamp_nsec + image_extension, rgb, compression_params);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to rgb.", rgb_msg->encoding.c_str());
    }//catch


    //DEPTH image
    try
    {
      //convert message too opencv mat
      depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image;
      cv::imshow("depth",depth );
      cv::resizeWindow("depth",432, 768);
      std::string timestamp_sec =std::to_string(rgb_msg->header.stamp.sec); 
      std::string timestamp_nsec=std::to_string(rgb_msg->header.stamp.nsec); 
      cv::imwrite(depth_save_path + timestamp_sec + "_" + timestamp_nsec  + image_extension, depth, compression_params);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to depth.", depth_msg->encoding.c_str());
    }//catch



    //RAW DEPTH image
    try
    {
      //convert message too opencv mat
      raw_depth = cv_bridge::toCvShare(raw_depth_msg, raw_depth_msg->encoding)->image;
      std::string timestamp_sec =std::to_string(rgb_msg->header.stamp.sec); 
      std::string timestamp_nsec=std::to_string(rgb_msg->header.stamp.nsec); 
      cv::imwrite(raw_depth_save_path + timestamp_sec + "_" + timestamp_nsec  + image_extension, raw_depth, compression_params);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to raw depth'.", raw_depth_msg->encoding.c_str());
    }//catch


    save_images = false;
  }//if save images
  lock.unlock();


}//callback



/*
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
      cv::resizeWindow("rgb",432, 768);
      
      cv::imwrite(rgb_save_path + std::to_string(counter) + image_extension, rgb, compression_params);
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
    ROS_INFO("DEPTH SAVE CALLBACK");
    try
    {
      depth = cv_bridge::toCvShare(msg, msg->encoding)->image;
      cv::imshow("depth",depth );
      cv::resizeWindow("depth",432, 768);
      
      saved = cv::imwrite( depth_save_path + std::to_string(counter) + image_extension, depth, compression_params);
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
      
      cv::imwrite(raw_depth_save_path  + std::to_string(counter) + image_extension, raw_depth, compression_params);

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


*/










bool save(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
 /* counter++;
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
 */ 

  lock.lock();
  save_images = true;
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
  ROS_INFO("SAVE SERVICE");
  lock.unlock();

  return true;
}


int main(int argc, char **argv)
{
/*  std::string node_name = "saver";
  if(argc > 1)
    node_name = std::string(argv[1]);
  ROS_INFO("name: %s", node_name.c_str());
*/

  
  ros::init(argc, argv, "saver",  ros::init_options::AnonymousName);

  ros::NodeHandle nh = ros::NodeHandle("~");
  nh.getParam("base_name", base_name);


  mkdir((base_save_path_1+ base_name + base_save_path_2).c_str(),0777);
  mkdir((base_save_path_1+ base_name + base_save_path_2 + rgb_save_path).c_str(),0777);
  mkdir((base_save_path_1+ base_name + base_save_path_2 + depth_save_path).c_str(),0777);
  mkdir((base_save_path_1+ base_name + base_save_path_2 + raw_depth_save_path).c_str(),0777);


  rgb_save_path = base_save_path_1 +  base_name  + base_save_path_2 + rgb_save_name;
  depth_save_path = base_save_path_1 +  base_name +  base_save_path_2 + depth_save_name;
  raw_depth_save_path = base_save_path_1 +  base_name + base_save_path_2 + raw_depth_save_name;


  ROS_INFO("%s", rgb_save_path.c_str());



  std::string ns = "/" + base_name;

  cv::namedWindow("rgb",CV_WINDOW_NORMAL);
  cv::namedWindow("depth", CV_WINDOW_NORMAL);
  cv::startWindowThread();

  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);

  image_transport::ImageTransport it(nh);



  image_transport::TransportHints rgb_hints("raw");
  image_transport::TransportHints depth_hints("raw");
  image_transport::TransportHints raw_depth_hints("raw");

  std::string rgb_topic = ns + K2_TOPIC_IMAGE_COLOR + K2_TOPIC_RAW;
  std::string depth_topic = ns + K2_TOPIC_HIRES_DEPTH + K2_TOPIC_RAW;
  std::string raw_depth_topic = ns + K2_TOPIC_IMAGE_DEPTH + K2_TOPIC_RAW;
  int queue_size = 3;

  rgb_filter_sub = new image_transport::SubscriberFilter(it, rgb_topic,queue_size, rgb_hints);
  depth_filter_sub = new image_transport::SubscriberFilter(it, depth_topic,queue_size, depth_hints);
  raw_depth_filter_sub = new image_transport::SubscriberFilter(it, raw_depth_topic,queue_size, raw_depth_hints);


  syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queue_size), *rgb_filter_sub, *depth_filter_sub,*raw_depth_filter_sub);
  syncExact->registerCallback(boost::bind(&images_callback, _1, _2 ,_3));

/*
  image_transport::Subscriber rgb_sub = it.subscribe(ns + K2_TOPIC_IMAGE_COLOR + K2_TOPIC_RAW, 1, rgb_callback);
  image_transport::Subscriber depth_sub = it.subscribe(ns + K2_TOPIC_HIRES_DEPTH + K2_TOPIC_RAW, 1, depth_callback);
  image_transport::Subscriber raw_depth_sub = it.subscribe(ns + K2_TOPIC_IMAGE_DEPTH + K2_TOPIC_RAW, 1, raw_depth_callback);

*/

  ros::ServiceServer service =  nh.advertiseService(ns + "/save_images", save);
//  ros::AsyncSpinner spinner(4);
 // spinner.start();
  //ros::waitForShutdown();
  while(ros::ok()){
    ros::spinOnce();
  } 
  cv::destroyWindow("rgb");
  cv::destroyWindow("depth");
  ros::shutdown();
}

