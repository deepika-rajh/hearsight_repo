/*****************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "InputStereoCameraROS.h"
#include <cv_bridge/cv_bridge.h>

const uint32_t kQueueSize = 1;
const uint32_t kSyncQueueSize = 3;
const char* kLeftImageTopic = "left_image";
const char* kRightImageTopic = "right_image";

InputStereoCameraROS::InputStereoCameraROS(ros::NodeHandle* node_handle, const std::string & config )
{
	left_image_sub_.reset(new ImageSubscriber(*node_handle, kLeftImageTopic, kQueueSize));
	right_image_sub_.reset(new ImageSubscriber(*node_handle, kRightImageTopic, kQueueSize));
	time_synchronizer_.reset(new TimeSynchronizer(SyncPolicy(kSyncQueueSize), *left_image_sub_, *right_image_sub_));
	time_synchronizer_->registerCallback(boost::bind(&InputStereoCameraROS::callback, this,
		/* boost::placeholders::*/_1, _2));
    ReadCameraConfig( config, cameraParams );
}

InputStereoCameraROS::~InputStereoCameraROS()
{

}

void InputStereoCameraROS::callback(const sensor_msgs::ImageConstPtr& left_image, 
	const sensor_msgs::ImageConstPtr& right_image)
{
	if (left_image != nullptr && right_image != nullptr)
    {
        std::cout << "left image time: "<< left_image->header.stamp
            << ", right image time: "<< right_image->header.stamp << std::endl;
    }
    else 
    {
        std::cout << "recieve empty" << std::endl;
		return;
    }
	
    cv_bridge::CvImagePtr cvL_ptr, cvR_ptr;
    try
    {
        cvL_ptr = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::MONO8);
        cvR_ptr = cv_bridge::toCvCopy(right_image, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
    }
	int imageRows = cvL_ptr->image.rows;
	int imageCols = cvL_ptr->image.cols;
	cv::Mat imagePair(imageRows*2, imageCols, CV_8UC1);
	cv::vconcat(cvL_ptr->image, cvR_ptr->image, imagePair);
	ros::Time t = left_image->header.stamp;
	callback_(t.toNSec(), imagePair.data, NULL);
}

void InputStereoCameraROS::ReadCameraConfig( const std::string & filename, rvCameraParams & cameraParams )
{
	printf("***ZYM*** %s\n",filename.c_str());
   cv::FileStorage fs_read( filename.c_str(), cv::FileStorage::READ );

   cameraParams.cameraType = rvStereo;

   cv::Size imageSize;
   fs_read["Image_Size"] >> imageSize;
   printf("***ZYM*** width %d height %d\n", imageSize.width, imageSize.height);
   cv::Mat cameraIntrinsics0, distortion0;
   fs_read["Camera_Matrix1"] >> cameraIntrinsics0;
   fs_read["Distortion_Coefficients1"] >> distortion0;
   getCameraSetting(cameraIntrinsics0, distortion0, imageSize, cameraParams.stereo.camera[0]);

   cv::Mat cameraIntrinsics1, distortion1;
   fs_read["Camera_Matrix2"] >> cameraIntrinsics1;
   fs_read["Distortion_Coefficients2"] >> distortion1;
   getCameraSetting(cameraIntrinsics1, distortion1, imageSize, cameraParams.stereo.camera[1]);

   cv::Mat rotation, r;
   fs_read["R"] >> rotation;

   cv::Mat t;
   fs_read["T"] >> t;
   t = t /1000.;

   if( t.at<double>( 0 ) > 0 )
   {
      rotation = rotation.inv();
      t = -rotation * t;
   }

   cv::Rodrigues(rotation, r);
   cameraParams.stereo.rotation[0] = (float32_t)r.at<double>(0);
   cameraParams.stereo.rotation[1] = (float32_t)r.at<double>(1);
   cameraParams.stereo.rotation[2] = (float32_t)r.at<double>(2);

   cameraParams.stereo.translation[0] = (float32_t)t.at<double>(0);
   cameraParams.stereo.translation[1] = (float32_t)t.at<double>(1);
   cameraParams.stereo.translation[2] = (float32_t)t.at<double>(2);

   cameraParams.stereoRect.camera[0].initialized = true;
   cameraParams.stereoRect.camera[0].pixelHeight = imageSize.height;
   cameraParams.stereoRect.camera[0].pixelWidth = imageSize.width;
   cameraParams.stereoRect.camera[1].initialized = true;
   cameraParams.stereoRect.camera[1].pixelHeight = imageSize.height;
   cameraParams.stereoRect.camera[1].pixelWidth = imageSize.width;

   cv::Mat R0, R1, P0, P1, Q;
   cv::stereoRectify( cameraIntrinsics0, distortion0, cameraIntrinsics1, distortion1,
                      imageSize, r, t, R0, R1, P0, P1, Q, cv::CALIB_ZERO_DISPARITY, 0 );

   for( size_t i = 0; i < 3; i++ )
   {
       for( size_t j = 0; j < 3; j++ )
       {
           cameraParams.stereoRect.camera[0].P[i][j] = P0.at<double>( i, j );
           cameraParams.stereoRect.camera[1].P[i][j] = P1.at<double>( i, j );
           cameraParams.stereoRect.camera[0].R[i][j] = R0.at<double>( i, j );
           cameraParams.stereoRect.camera[1].R[i][j] = R1.at<double>( i, j );
       }
       cameraParams.stereoRect.camera[0].P[i][3] = P0.at<double>( i, 3 );
       cameraParams.stereoRect.camera[1].P[i][3] = P1.at<double>( i, 3 );
   }
   cameraParams.stereoRect.translation[0] = cameraParams.stereoRect.camera[1].P[0][3] / cameraParams.stereoRect.camera[1].P[0][0];
   cameraParams.stereoRect.translation[1] = cameraParams.stereoRect.translation[2] = 0;
}

void InputStereoCameraROS::getCameraSetting(const cv::Mat & intrinsics, const cv::Mat & distortion, const cv::Size & imageSize, rvCameraIntrinsic & cameraConfig)
{
   cameraConfig.focalLength[0] = (float32_t)intrinsics.at<double>( 0, 0 );
   cameraConfig.focalLength[1] = (float32_t)intrinsics.at<double>( 1, 1 );
   cameraConfig.principalPoint[0] = (float32_t)intrinsics.at<double>( 0, 2 );
   cameraConfig.principalPoint[1] = (float32_t)intrinsics.at<double>( 1, 2 );

   cameraConfig.pixelWidth = imageSize.width;
   cameraConfig.pixelHeight = imageSize.height;

   memset( cameraConfig.distortion, 0, sizeof( cameraConfig.distortion ) );
   switch(distortion.cols)
   {
       case 5:
           cameraConfig.distortionModel = rvDistortionModel::Polynomial5;
		   break;
       case 4:
           cameraConfig.distortionModel = rvDistortionModel::Polynomial4;
		   break;
       default:
       case 8:
           cameraConfig.distortionModel = rvDistortionModel::RationalModel8;
		   break;
   }
   for (int i=0; i<distortion.cols; i++ )
      cameraConfig.distortion[i] = (float32_t)distortion.at<double>( i );
}
