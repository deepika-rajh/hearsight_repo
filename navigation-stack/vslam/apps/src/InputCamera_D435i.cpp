/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <thread>
#include <functional>

#include "SystemTime.h"
#include "InputCamera_D435i.h"

using namespace std;
#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 360

#ifdef OPENCV_TEST
#include <opencv2/opencv.hpp>
using namespace cv;
#endif

#ifdef ROS_BASED
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
using namespace cv;

extern rclcpp::Node::SharedPtr g_node;
//extern image_transport::Publisher    color_pub;
extern image_transport::Publisher    gray_pub;
extern image_transport::Publisher    depth_pub;
extern rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub;


void publishDepth(rs2::depth_frame depth_frame, const rclcpp::Time & t)
{
    auto depth_image = cv::Mat(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16UC1,
        const_cast<void *>(depth_frame.get_data()), cv::Mat::AUTO_STEP);

    sensor_msgs::msg::Image::SharedPtr img;
    img = cv_bridge::CvImage(
      std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth_image).toImageMsg();

    auto bpp = depth_frame.get_bytes_per_pixel();
    auto height = depth_frame.get_height();
    auto width = depth_frame.get_width();
    img->width = width;
    img->height = height;
    img->is_bigendian = false;
    img->step = width * bpp;
    img->header.frame_id = "depth_frame";
    img->header.stamp = t;
    depth_pub.publish(img);
}
  
void publishColor(rs2::frame f, const rclcpp::Time & t)
{
    auto width = 0;
    auto height = 0;
    auto bpp = 1;
    if (f.is<rs2::video_frame>()) {
        auto image = f.as<rs2::video_frame>();
        width = image.get_width();
        height = image.get_height();
        bpp = image.get_bytes_per_pixel();
    }
    else
    {
        return;
    }

    auto color_img = cv::Mat(cv::Size(width, height), CV_8UC3,
        const_cast<void *>(f.get_data()), cv::Mat::AUTO_STEP);
    //cv::Mat gray_img;
    //cvtColor(color_img, gray_img, COLOR_RGB2GRAY);

    sensor_msgs::msg::Image::SharedPtr img;
    //img = cv_bridge::CvImage(
    //  std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, gray_img).toImageMsg();
    img = cv_bridge::CvImage(
    std_msgs::msg::Header(), sensor_msgs::image_encodings::RGB8, color_img).toImageMsg();
	
    img->width = width;
    img->height = height;
    img->is_bigendian = false;
    img->step = width * bpp;
    img->header.frame_id = "color_frame";
    img->header.stamp = t;
    //color_pub.publish(img);
}

void publishGray(unsigned char* buf, const rclcpp::Time & t)
{
    Mat gray_image(Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
    memcpy(gray_image.data, buf, IMAGE_WIDTH * IMAGE_HEIGHT);

    sensor_msgs::msg::Image::SharedPtr img;
    img = cv_bridge::CvImage(
      std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, gray_image).toImageMsg();
	  
    img->width = IMAGE_WIDTH;
    img->height = IMAGE_HEIGHT;
    img->is_bigendian = false;
    img->step = IMAGE_WIDTH;
    img->header.frame_id = "gray_frame";
    img->header.stamp = t;
    gray_pub.publish(img);
}



void publishRGBCameraInfo(const rs2_intrinsics & intrinsic, const rclcpp::Time & t)
{
    sensor_msgs::msg::CameraInfo info_msg;

    info_msg.width = intrinsic.width;
    info_msg.height = intrinsic.height;
    info_msg.header.frame_id = "camera_info";
    info_msg.header.stamp = t;
	
    info_msg.distortion_model = "plumb_bob";
    
    info_msg.k.at(0) = intrinsic.fx;
    info_msg.k.at(2) = intrinsic.ppx;
    info_msg.k.at(4) = intrinsic.fy;
    info_msg.k.at(5) = intrinsic.ppy;
    info_msg.k.at(8) = 1;

    info_msg.p.at(0) = intrinsic.fx;
    info_msg.p.at(1) = 0;
    info_msg.p.at(2) = intrinsic.ppx;
    info_msg.p.at(3) = 0;
    info_msg.p.at(4) = 0;
    info_msg.p.at(5) = intrinsic.fy;
    info_msg.p.at(6) = intrinsic.ppy;
    info_msg.p.at(7) = 0;
    info_msg.p.at(8) = 0;
    info_msg.p.at(9) = 0;
    info_msg.p.at(10) = 1;
    info_msg.p.at(11) = 0;

    for (int i = 0; i < 5; i++) {
       info_msg.d.push_back(intrinsic.coeffs[i]);
    }

    cam_info_pub->publish(info_msg);
}
#endif

InputCamera_D435i::InputCamera_D435i()
{
   running = false;
   callback = NULL;
   clockOffset = 0;

   findClocksOffsetForCamera();
}

InputCamera_D435i::~InputCamera_D435i()
{
   printf( "release camera!\n" );
}


void InputCamera_D435i::findClocksOffsetForCamera()
{
   realClock = getRealTime();
   monotonicClock = getMonotonicTime();
   clockOffset = realClock - monotonicClock;
   //printf( "findClocksOffsetForCamera realClock = %" PRId64 ", monotonicClock=%" PRId64 ", clockOffset=%" PRId64 " \n ", realClock, monotonicClock, clockOffset );
}


void InputCamera_D435i::proc()
{
   rs2::frameset frames;
   rs2::align align(RS2_STREAM_COLOR);
   unsigned char buf[IMAGE_WIDTH * IMAGE_HEIGHT];

   #ifdef OPENCV_TEST
   const auto window_color = "Color Image";
   namedWindow(window_color, WINDOW_AUTOSIZE);
   const auto window_gray = "Gary Image";
   namedWindow(window_gray, WINDOW_AUTOSIZE);
   #endif

   #ifdef ROS_BASED
   double camera_time_base;
   bool init_base = false;
   rclcpp::Time ros_time_base;
   rclcpp::Clock ros_clock(RCL_ROS_TIME);
   uint64_t elapsed_camera_ns;
   
   rclcpp::Time t;
   #endif

   while (running )
   {
      frames = pipe.wait_for_frames();
      frames = align.process(frames);

      //Get each frame
      rs2::frame color_frame = frames.get_color_frame();
      rs2::depth_frame depth_frame = frames.get_depth_frame();

      //float dist_to_center = depth_frame.get_distance(IMAGE_WIDTH / 2, IMAGE_HEIGHT / 2);
      //std::cout << "The camera is facing an object " << dist_to_center << " meters away " << std::endl;

#ifdef OPENCV_TEST
      Mat color_image(Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
      Mat gray_image(Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
      unsigned char *pCData=(unsigned char*)color_frame.get_data();

      unsigned int v;
      for(int i=0; i<IMAGE_WIDTH*IMAGE_HEIGHT; ++i)
      {
         v = pCData[3*i] * 299 + pCData[3*i + 1] * 587 + pCData[3*i + 2] * 114 + 500;      
         v = v / 1000;
         buf[i]= (unsigned char)v;
         gray_image.data[i] = (unsigned char)v;
      }
      imshow(window_color, color_image);
      imshow(window_gray, gray_image);
      waitKey(1);
#else
	
	  unsigned char *pCData=(unsigned char*)color_frame.get_data();
      unsigned int v;
      for(int i=0; i<IMAGE_WIDTH*IMAGE_HEIGHT; ++i)
      {
         v = pCData[3*i] * 299 + pCData[3*i + 1] * 587 + pCData[3*i + 2] * 114 + 500;      
         v = v / 1000;
         buf[i]= (unsigned char)v;
      }
#endif

      //in ms, so ms * 1000000 to ns
      callback( (int64_t)(color_frame.get_timestamp()* 1000000), buf, (uint16_t *)depth_frame.get_data() );
      
      #ifdef ROS_BASED
      if (false == init_base) {
         init_base = true;
         ros_time_base = ros_clock.now();
         camera_time_base = color_frame.get_timestamp();
      }
	  
      elapsed_camera_ns = (color_frame.get_timestamp() - camera_time_base) * 1000000;  // ms * 1000000
      t = rclcpp::Time(ros_time_base.nanoseconds() + elapsed_camera_ns, RCL_ROS_TIME);
		  
      publishDepth(depth_frame, t);
      //publishColor(color_frame, t);
      publishGray(buf, t);
      publishRGBCameraInfo(intrinsics, t);
      #endif
   }
}


bool InputCamera_D435i::start()
{
   running = true;
   printf("**** luow ******. start camera.\n");
   //config stream
   cfg.enable_stream(RS2_STREAM_COLOR, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_RGB8, 15);
   //cfg.enable_stream(RS2_STREAM_INFRARED, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_Y8, 30);
   cfg.enable_stream(RS2_STREAM_DEPTH, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_Z16, 15);

   //enable config
   profiles = pipe.start(cfg);
   
   rs2::stream_profile color_profile = profiles.get_stream(RS2_STREAM_COLOR);
   if (auto video_profile = color_profile.as<rs2::video_stream_profile>())
   {
      try {
         intrinsics = video_profile.get_intrinsics();
                  
		 printf("intrinsics: ppx %.4f, ppy %.4f, fx %.4f, fy %.4f, model %d\n",
			  intrinsics.ppx, intrinsics.ppy,
			  intrinsics.fx, intrinsics.fy, intrinsics.model);
		 printf("intrinsics coeffs: %.4f, %.4f, %.4f, %.4f, %.4f\n",
              intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2],
              intrinsics.coeffs[3], intrinsics.coeffs[4]);
              configuration.inputPixelWidth = intrinsics.width;
              configuration.inputPixelHeight = intrinsics.height;
	      configuration.outputPixelWidth = intrinsics.width;
	      configuration.outputPixelHeight = intrinsics.height;

              memset(configuration.inputCameraMatrix, 0, sizeof(configuration.inputCameraMatrix));
              configuration.inputCameraMatrix[0] = intrinsics.fx;
              configuration.inputCameraMatrix[2] = intrinsics.ppx;
              configuration.inputCameraMatrix[4] = intrinsics.fy;
              configuration.inputCameraMatrix[5] = intrinsics.ppy;
              configuration.inputCameraMatrix[8] = 1.f;
              memcpy(configuration.outputCameraMatrix, configuration.inputCameraMatrix, sizeof(configuration.inputCameraMatrix));
              configuration.distortionModel = rvCameraParams::NoDistortion;
              memset( configuration.distortionCoefficient, 0, sizeof( configuration.distortionCoefficient ) );
              configuration.distortionCoefficient[0] = intrinsics.coeffs[0];
              configuration.distortionCoefficient[1] = intrinsics.coeffs[1];
              configuration.distortionCoefficient[2] = intrinsics.coeffs[2];
              configuration.distortionCoefficient[3] = intrinsics.coeffs[3];
              configuration.distortionCoefficient[4] = intrinsics.coeffs[4];

	  }
	  catch (const std::exception& e) {
	     printf("failed to get camera intrinsic\n");
	  }
   }

   thread = std::make_shared<std::thread>(std::mem_fn(&InputCamera_D435i::proc), this);
   return true;
}


bool InputCamera_D435i::stop()
{
   running = false;
   if(thread)
      thread->join();

   return true;
}

void InputCamera_D435i::addCallback( CameraCallback _callback )
{
   callback = _callback;
}

const rvCameraParams & InputCamera_D435i::getCameraConfiguration( ) const
{
   return configuration;
}
