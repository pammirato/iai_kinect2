





#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>

#include <dirent.h> 

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>

#include <kinect2_definitions.h>
#include <depth_registration.h>


int main(int agrc, char ** argv)
{

  std::string load_depth_path = "/home/ammirato/Data/Test1/unreg_depth/";
  std::string save_depth_path = "/home/ammirato/Data/Test1/raw_depth/";
 
  mkdir((save_depth_path).c_str(),0777);
 
  std::string ns = "K1";

  std::string calibPath ="/home/ammirato/catkin_ws/src/iai_kinect2/kinect2_bridge/data/500317541942/"; 


  std::string color_calib_filename = calibPath + K2_CALIB_COLOR;
  std::string ir_calib_filename = calibPath + K2_CALIB_IR;
  std::string pose_calib_filename = calibPath + K2_CALIB_POSE;
  std::string depth_calib_filename = calibPath + K2_CALIB_DEPTH;


  std::vector<int> compressionParams;
  compressionParams.resize(7, 0);
  compressionParams[0] = CV_IMWRITE_JPEG_QUALITY;
  compressionParams[1] = 100;
  compressionParams[2] = CV_IMWRITE_PNG_COMPRESSION;
  compressionParams[3] = 0;
  compressionParams[4] = CV_IMWRITE_PNG_STRATEGY;
  compressionParams[5] = CV_IMWRITE_PNG_STRATEGY_RLE;
  compressionParams[6] = 0; 


//depth = cv::Mat(depthFrame->height, depthFrame->width, CV_32FC1, depthFrame->data);
  cv::Mat rotation, translation;
  cv::Mat cameraMatrixColor, distortionColor, cameraMatrixLowRes, cameraMatrixIr, distortionIr;
  cv::Size sizeColor(1920,1080);
  cv::Size sizeIr(512,424);
  double depthShift;

  cameraMatrixColor = cv::Mat::eye(3, 3, CV_64F);
  distortionColor = cv::Mat::zeros(1, 5, CV_64F);
 
  cameraMatrixIr = cv::Mat::eye(3, 3, CV_64F);
  distortionIr = cv::Mat::zeros(1, 5, CV_64F);
  
  rotation = cv::Mat::eye(3, 3, CV_64F);
  translation = cv::Mat::zeros(3, 1, CV_64F);
  translation.at<double>(0) = -0.0520;



  cv::FileStorage fs;

  //COLOR
  if(fs.open(color_calib_filename, cv::FileStorage::READ))
  {
    fs[K2_CALIB_CAMERA_MATRIX] >> cameraMatrixColor;
    fs[K2_CALIB_DISTORTION] >> distortionColor;
    fs.release();
  }
  else
  {
    std::cerr << "can't open calibration file: " << color_calib_filename << std::endl;
    return false;
  }


  //IR
  if(fs.open(ir_calib_filename, cv::FileStorage::READ))
  {
    fs[K2_CALIB_CAMERA_MATRIX] >> cameraMatrixIr;
    fs[K2_CALIB_DISTORTION] >> distortionIr;
    fs.release();
  }
  else
  {
    std::cerr << "can't open calibration file: " << ir_calib_filename << std::endl;
    return false;
  }


  //POSE
  if(fs.open(pose_calib_filename, cv::FileStorage::READ))
  {
    fs[K2_CALIB_ROTATION] >> rotation;
    fs[K2_CALIB_TRANSLATION] >> translation;
    fs.release();
  }
  else
  {
    std::cerr << "can't open calibration pose file: " << pose_calib_filename << std::endl;
    return false;
  }


  //DPETH
  if(fs.open(depth_calib_filename, cv::FileStorage::READ))
  {
    fs[K2_CALIB_DEPTH_SHIFT] >> depthShift;
    fs.release();
  }
  else
  {
    std::cerr << "can't open calibration depth file: " << depth_calib_filename << std::endl;
    return false;
  }


















  DepthRegistration *depthRegHighRes;

  depthRegHighRes = DepthRegistration::New(DepthRegistration::CPU);

  //depthRegHighRes->init(cameraMatrixColor, sizeColor, cameraMatrixIr, sizeIr, distortionIr, rotation, translation, 0.5f, maxDepth, device);
  depthRegHighRes->init(cameraMatrixColor, sizeColor, cameraMatrixIr, sizeIr, distortionIr, rotation, translation, 0.5f, 12.0f);


  cv::Mat unreg_depth,depthShifted, reg_depth;




  //int counter = 0;
  size_t first_h, first_period;
  std::string index_string;

  DIR *d;
  struct dirent *dir;
  d = opendir(load_depth_path.c_str());
  if (d)
  {
    while ((dir = readdir(d)) != NULL)
    {
      std::string filename (dir->d_name);

      if(filename.compare(".") == 0 || filename.compare("..") == 0)
      {
        continue;
      }
      printf("%s\n", dir->d_name);

      first_h = filename.find_first_of("h");
      first_period = filename.find_first_of(".");
      index_string = filename.substr(first_h+1,first_period-first_h-1);     





      unreg_depth = cv::imread(load_depth_path + filename, CV_LOAD_IMAGE_ANYDEPTH);  
      
      unreg_depth.convertTo(depthShifted, CV_16U, 1, depthShift);
      cv::flip(depthShifted, depthShifted, 1); 
      depthRegHighRes->registerDepth(depthShifted, reg_depth);

      cv::imwrite(save_depth_path + "depth" + index_string + ns + ".png",
                  reg_depth);


    }//while readdir

    closedir(d);
  }//if d

}//main
