/*****************************************************************************
@copyright
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "InputCamera_Kinect2.h"
#include "opencv2/opencv.hpp"
#include <fstream>

const uint32_t kQueueSize = 1;
const uint32_t kSyncQueueSize = 3;
const char* kLeftImageTopic = "left_image";
const char* kDepthImageTopic = "depth_image";

InputCamera_Kinect2::InputCamera_Kinect2(ros::NodeHandle* node_handle, const std::string & config)
{
    left_image_sub_.reset(new ImageSubscriber(*node_handle, kLeftImageTopic, kQueueSize));
    depth_image_sub_.reset(new ImageSubscriber(*node_handle, kDepthImageTopic, kQueueSize));
    time_synchronizer_.reset(new TimeSynchronizer(SyncPolicy(kSyncQueueSize), *left_image_sub_, *depth_image_sub_));
    time_synchronizer_->registerCallback(boost::bind(&InputCamera_Kinect2::callback, this,
		/* boost::placeholders::*/_1, _2));
    
    readCameraParameter( config, cameraParams );
}

InputCamera_Kinect2::~InputCamera_Kinect2()
{
   printf( "release camera!\n" );
}


void InputCamera_Kinect2::callback(const sensor_msgs::ImageConstPtr& left_image, 
	const sensor_msgs::ImageConstPtr& depth_image)
{
    if (left_image != nullptr && depth_image != nullptr)
    {
        std::cout << "left image time: "<< left_image->header.stamp
            << ", depth image time: "<< depth_image->header.stamp << std::endl;
    }
    else 
    {
        std::cout << "recieve empty" << std::endl;
		return;
    }

    cv::Mat colorImage(cameraParams.camera.pixelHeight, cameraParams.camera.pixelWidth, CV_8UC3); 
    memcpy(colorImage.data, left_image->data.data(), cameraParams.camera.pixelHeight*cameraParams.camera.pixelWidth*3);//
    ros::Time t = left_image->header.stamp;
    cv::Mat leftImage;
    cv::cvtColor(colorImage, leftImage, cv::COLOR_BGR2GRAY);
    cv::Mat depthImage(cameraParams.camera.pixelHeight, cameraParams.camera.pixelWidth, CV_16UC1);
    memcpy(depthImage.data, depth_image->data.data(), cameraParams.camera.pixelHeight*cameraParams.camera.pixelWidth*2);//

    callback_(t.toNSec(), leftImage.data, (uint16_t *)depthImage.data);
}

const rvCameraParams & InputCamera_Kinect2::getCameraConfiguration( ) const
{
   return cameraParams;
}


bool ReadMatrix( std::ifstream & file, float32_t * matrix, size_t bufLength )
{
   std::string line, valName;
   int rows = 0, cols = 0;

   std::getline( file, line );
   std::istringstream issRow( line );
   issRow >> valName >> rows;

   std::getline( file, line );
   std::istringstream issCol( line );
   issCol >> valName >> cols;
   
   if (cols * rows > bufLength)
      return false;

   std::getline( file, line );
   size_t index = line.find_first_of( '[' );
   size_t length = line.length();
   std::string matrixStr = line.substr( index + 1, length - index - 1 );
   while( std::getline( file, line ) )
   {
      size_t index1 = line.find_first_of( ']' );
      if( index1 == std::string::npos )
      {
         matrixStr = matrixStr + line;
      }
      else
      {
         matrixStr = matrixStr + line.substr( 0, index1 );
         break;
      }
   }

   std::istringstream iss( matrixStr );

   std::string numberStr;
   for( int i = 0; i < rows * cols; i++ )
   {
      iss >> matrix[i] >> valName;
   }
}


bool InputCamera_Kinect2::readCameraParameter( const std::string & filename, rvCameraParams & cameraParams )
{  
   std::ifstream cfg( filename, std::ifstream::in );
   if( !cfg.is_open() )
   {
      printf( "Fail to open configuration file: %s\n", filename.c_str() );
      return false;
   }

   std::string line;
   std::string itemName;
   float32_t matrix[12];
   while( std::getline( cfg, line ) )
   {
      if( line.length() == 0 )
      {
         continue;
      }
      if( line[0] == '#' )
      {
         continue;
      }
      std::istringstream iss( line );
      iss >> itemName;
      if( itemName.compare( "image_width:" ) == 0 )
      {
         iss >> cameraParams.camera.pixelWidth;
      }
      else if( itemName.compare( "image_height:" ) == 0 )
      {
         iss >> cameraParams.camera.pixelHeight;
      }
      else if( itemName.compare( "camera_matrix:" ) == 0 )
      {
         ReadMatrix( cfg, matrix, 12 );
         cameraParams.camera.focalLength[0] = matrix[0];
         cameraParams.camera.focalLength[1] = matrix[4];
         cameraParams.camera.principalPoint[0] = matrix[2];
         cameraParams.camera.principalPoint[1] = matrix[5];
      }
      else if( itemName.compare( "projection_matrix:" ) == 0 )
      {
         ReadMatrix( cfg, matrix, 12 );
         for (size_t i = 0, k = 0; i<3; i++)
         {
             for (size_t j=0; j<4; j++, k++)
             {
                 cameraParams.cameraRect.P[i][j] = matrix[k];
             }
         }
      }
      else if( itemName.compare( "rectification_matrix:" ) == 0 )
      {
         ReadMatrix( cfg, matrix, 12 );
         for (size_t i = 0, k=0; i<3; i++)
         {
             for (size_t j=0; j<3; j++, k++)
             {
            	 cameraParams.cameraRect.R[i][j] = matrix[k];
             }
         }
      }
      else if( itemName.compare( "project_image_width:" ) == 0 )
      {
         iss >> cameraParams.cameraRect.pixelWidth;
      }
      else if( itemName.compare( "project_image_height:" ) == 0 )
      {
         iss >> cameraParams.cameraRect.pixelHeight;
      }
   }
   cameraParams.camera.distortionModel = rvDistortionModel::NoDistortion;
   cameraParams.cameraType = rvGrayDepth;
   cameraParams.camera.memoryStride = cameraParams.camera.pixelWidth;
   cameraParams.camera.uvOffset = 0;   
   cameraParams.cameraRect.initialized = true;
   
   return true;
}
