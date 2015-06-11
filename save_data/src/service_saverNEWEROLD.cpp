#include <service_saver.h>



namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}








Receiver::Receiver(const std::string &topicColor, const std::string &topicDepth)
{
  queue_size = 1;
  topic_color = topicColor;
  topic_depth = topicDepth;

//  nh = ros::NodeHandle("~");
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(topic_color,1,&Receiver::image_callback, this);  

}

void Receiver::run()
{
  cv::namedWindow("view");
  cv::startWindowThread();
  ros::spin();
  cv::destroyWindow("view");


}

void Receiver::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
  cv::waitKey(30);

}





int main(int argc, char **argv)
{

  ros::init(argc, argv, "saver", ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    return 0;
  }


  std::string ns = K2_DEFAULT_NS;
  std::string topicColor = K2_TOPIC_LORES_COLOR K2_TOPIC_RAW;
  std::string topicDepth = K2_TOPIC_LORES_DEPTH K2_TOPIC_RAW;

  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;

  std::cout << "topic color: " << topicColor << std::endl;
  std::cout << "topic depth: " << topicDepth << std::endl;

  Receiver receiver(topicColor, topicDepth);

  std::cout << "starting receiver..." << std::endl;
  receiver.run();

  ros::shutdown();
  return 0;


}
