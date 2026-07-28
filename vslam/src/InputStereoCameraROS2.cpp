/*****************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "InputStereoCameraROS2.h"
#include <cv_bridge/cv_bridge.h>


const uint32_t kQueueSize = 1;
const uint32_t kSyncQueueSize = 3;
const char* kLeftImageTopic = "/left/image_raw";
const char* kRightImageTopic = "/right/image_raw";

InputStereoCameraROS2::InputStereoCameraROS2(rclcpp::Node::SharedPtr const &node_, const std::string & config ): node(node_)
{
	cameraParams.cameraType = rvStereo;
    cameraParams.imageFormat = Y_ONLY_FORMAT;
    cameraParams.cameraType = rvStereo;

	if (ReadCameraConfig(config, cameraParams))
	{
		printf("[stereo-vslam] loaded camera calibration from '%s' (%dx%d)\n",
		       config.c_str(),
		       cameraParams.stereo.camera[0].pixelWidth,
		       cameraParams.stereo.camera[0].pixelHeight);
		gotLeftCameraPara = true;
		gotRightCameraPara = true;
		leftInfoSub = NULL;
		rightInfoSub = NULL;

		left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(node, kLeftImageTopic);
        right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(node, kRightImageTopic);
		makeSncPolocy();

		// Compute depth ourselves from the stereo pair (via rvDFS) and feed the
		// engine as rvGrayDepth instead of genuine rvStereo - see the comment on
		// dfsHandle in InputStereoCameraROS2.h for why.
		cameraParams.cameraType = rvGrayDepth;
		initDFS();
	}
	else
	{
        printf("[stereo-vslam] calibration '%s' not loaded; auto-detecting resolution + intrinsics from /left/camera_info and /right/camera_info\n",
               config.c_str());
        leftInfoSub = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
        std::string("/left/camera_info"), 10, bind(&InputStereoCameraROS2::leftInfo_callback, this, std::placeholders::_1));
        rightInfoSub = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
        std::string("/right/camera_info"), 10, bind(&InputStereoCameraROS2::rightInfo_callback, this, std::placeholders::_1));

        gotLeftCameraPara = false;
        gotRightCameraPara = false;

        syncApproximate = NULL;
        left_sub = NULL;
        right_sub = NULL;
	}
}

InputStereoCameraROS2::~InputStereoCameraROS2()
{
    if (dfsHandle)
    {
        rvDFS_DeinitializeU16(dfsHandle);
        dfsHandle = nullptr;
    }
}

void InputStereoCameraROS2::callback(const sensor_msgs::msg::Image::ConstSharedPtr& left,
	const sensor_msgs::msg::Image::ConstSharedPtr& right)
{
    printf("===== STEREO CALLBACK ENTERED =====\n");
    if ( left != nullptr )
    {
		printf("Left  stamp : %u.%09u\n", left->header.stamp.sec, left->header.stamp.nanosec);
		printf("Right stamp : %u.%09u\n", right->header.stamp.sec, right->header.stamp.nanosec);
		printf("image time %10.6f\n",left->header.stamp.sec + left->header.stamp.nanosec*1e-9);
    }
    else
    {
        std::cout << "recieve empty" << std::endl;
		return;
    }

    printf("Wire size - left: %ux%u encoding=%s step=%u  |  right: %ux%u encoding=%s step=%u\n",
        left->width, left->height, left->encoding.c_str(), left->step,
        right->width, right->height, right->encoding.c_str(), right->step);

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrLeft = cv_bridge::toCvCopy(left, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
       printf("left error: %s\n", e.what());
       return;
    }

	try
    {
        cv_ptrRight = cv_bridge::toCvCopy(right, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
       printf("right error: %s\n", e.what());
       return;
    }

    // -------- DEBUG START --------
    printf("Left  cv::Mat: %dx%d type=%d  |  Right cv::Mat: %dx%d type=%d\n",
        cv_ptrLeft->image.cols, cv_ptrLeft->image.rows, cv_ptrLeft->image.type(),
        cv_ptrRight->image.cols, cv_ptrRight->image.rows, cv_ptrRight->image.type());
    printf("Calibration expects: left %ux%u  right %ux%u (mismatch here silently corrupts the "
        "DFS depth computation below)\n",
        cameraParams.stereo.camera[0].pixelWidth, cameraParams.stereo.camera[0].pixelHeight,
        cameraParams.stereo.camera[1].pixelWidth, cameraParams.stereo.camera[1].pixelHeight);

    double leftMin, leftMax, rightMin, rightMax;
    cv::minMaxLoc(cv_ptrLeft->image, &leftMin, &leftMax);
    cv::minMaxLoc(cv_ptrRight->image, &rightMin, &rightMax);
    cv::Scalar leftMean = cv::mean(cv_ptrLeft->image);
    cv::Scalar rightMean = cv::mean(cv_ptrRight->image);
    printf("Left  pixel range = %.0f - %.0f, mean = %.2f\n", leftMin, leftMax, leftMean[0]);
    printf("Right pixel range = %.0f - %.0f, mean = %.2f\n", rightMin, rightMax, rightMean[0]);
    // -------- DEBUG END --------

    if (!dfsHandle)
    {
        printf("DFS not initialized - skipping frame (no depth available)\n");
        return;
    }

    rvDFSInputParam in{};
    in.meta.version = 0x00010000;
    in.meta.paramSize = sizeof(rvDFSInMeta);
    in.meta.numParams = 0;
    in.meta.dfsParam = nullptr;
    in.meta.poseCameraInWorld = nullptr;
    in.inColor = nullptr;
    in.inV1.imgTimestamp = (uint64_t)left->header.stamp.sec * 1000000000ULL + left->header.stamp.nanosec;
    in.inV1.imgLeft = cv_ptrLeft->image.data;
    in.inV1.ionFDLeft = -1;
    in.inV1.imgRight = cv_ptrRight->image.data;
    in.inV1.ionFDRight = -1;

    rvDFSOutputParam out{};
    out.outV1.imgL = nullptr;
    out.outV1.imgR = nullptr;
    out.outV1.rectL = nullptr;
    out.outV1.rectR = nullptr;
    out.outV1.mapOfDisparity = nullptr;
    out.outV1.mapOfDepth = depthBuf.data();
    out.outV1.numPoints = 0;
    out.outV1.pointBuffer = nullptr;

    // -------- DFS DEBUG START --------
    if (!rvDFS_ComputeU16(dfsHandle, &in, &out))
    {
        printf("rvDFS_ComputeU16 FAILED - skipping frame\n");
        return;
    }
    uint16_t depthMin = 65535, depthMax = 0;
    double depthSum = 0;
    size_t validCount = 0;
    for (uint16_t d : depthBuf)
    {
        if (d == 0) continue;
        if (d < depthMin) depthMin = d;
        if (d > depthMax) depthMax = d;
        depthSum += d;
        validCount++;
    }
    printf("DFS depth: valid=%zu/%zu range=%u-%u mean=%.0fmm\n",
        validCount, depthBuf.size(), validCount ? depthMin : 0, depthMax,
        validCount ? depthSum / validCount : 0.0);
    // -------- DFS DEBUG END --------

    printf("call callback\n");
    rclcpp::Time t = left->header.stamp;

    printf("Calling VSLAM callback...\n");
    callback_(t.nanoseconds(), cv_ptrLeft->image.data, depthBuf.data());
    printf("Returned from VSLAM callback.\n");
}

bool InputStereoCameraROS2::initDFS()
{
    rvDFSParameter dfsParam{};
    dfsParam.version = 0x00010000;
    dfsParam.paramSize = sizeof(rvDFSParameter);
    dfsParam.inputSize = { (int)cameraParams.stereo.camera[0].pixelWidth,
                           (int)cameraParams.stereo.camera[0].pixelHeight,
                           (int)cameraParams.stereo.camera[0].pixelStride };
    dfsParam.imgFormat = Y_ONLY_FORMAT;
    dfsParam.inType = RV_DFS_IN_V1;
    dfsParam.outputSize = dfsParam.inputSize;
    dfsParam.mode = RV_DFS_BALANCE; // OpenCL - prefer Qualcomm HW accel over a CPU-only stereo matcher
    dfsParam.useDisp = false;       // use depthRange (mm) rather than raw disparity levels
    dfsParam.depthRange.minDepth = 300;  // mm
    dfsParam.depthRange.maxDepth = 5000; // mm - keep in sync with stereoSlam.cfg's depthFilter
    dfsParam.doRectification = false; // D455 infra streams are already factory-rectified
    dfsParam.ppLevel = RV_DFS_PP_BASIC;

    dfsHandle = rvDFS_InitializeU16(dfsParam, cameraParams.stereo);
    if (!dfsHandle)
    {
        printf("[stereo-vslam] rvDFS_InitializeU16 failed with RV_DFS_BALANCE (OpenCL) - retrying RV_DFS_COVERAGE (CPU)\n");
        dfsParam.mode = RV_DFS_COVERAGE;
        dfsHandle = rvDFS_InitializeU16(dfsParam, cameraParams.stereo);
    }

    if (dfsHandle)
    {
        depthBuf.assign((size_t)cameraParams.stereo.camera[0].pixelWidth *
                         cameraParams.stereo.camera[0].pixelHeight, 0);
        printf("[stereo-vslam] DFS initialized (mode=%s)\n",
               dfsParam.mode == RV_DFS_COVERAGE ? "COVERAGE/CPU" : "BALANCE/OpenCL");
    }
    else
    {
        printf("[stereo-vslam] DFS initialization FAILED entirely - no depth will be computed, "
               "every frame will be skipped!\n");
    }
    return dfsHandle != nullptr;
}

void InputStereoCameraROS2::leftInfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr rgbInfo)                        
{
    if (gotLeftCameraPara)
        return;
   
   printf("process camera info\n");
 
   copyInfo(rgbInfo, cameraParams.stereo.camera[0]);
   copyInfo(rgbInfo, cameraParams.stereoRect.camera[0]);
   gotLeftCameraPara = true;
   
   left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(node, kLeftImageTopic);
   
    if (gotLeftCameraPara && gotRightCameraPara)
    {
        makeSncPolocy();
    }
	
   leftInfoSub = NULL;

}



void InputStereoCameraROS2::rightInfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr rgbInfo)                        
{
    if (gotRightCameraPara)
        return;
   
   printf("*********process camera info*************\n");
 
   copyInfo(rgbInfo, cameraParams.stereo.camera[1]);
   copyInfo(rgbInfo, cameraParams.stereoRect.camera[1]);   
   gotRightCameraPara = true;
   
   right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(node, kRightImageTopic);
   
    if (gotLeftCameraPara && gotRightCameraPara)
    {
        makeSncPolocy();
    }
	
   rightInfoSub = NULL;

}


void
InputStereoCameraROS2::makeSncPolocy()
{
    printf("************sync polocy******************\n");
    syncApproximate = std::make_shared<StereoSync>(StereoSync(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(bind(&InputStereoCameraROS2::callback, this, std::placeholders::_1, std::placeholders::_2));
    printf("*********left_sub %p, right_sub %p*********\n", (void *)left_sub.get(), (void *)right_sub.get());
}


void 
InputStereoCameraROS2::copyInfo(const sensor_msgs::msg::CameraInfo::SharedPtr rgbInfo, rvRectCameraConfiguration &camera)
{
    camera.initialized = true;
   
    camera.pixelHeight = rgbInfo->height;
    camera.pixelWidth = rgbInfo->width;
   
    for (size_t i=0; i<3; ++i)
    {
         for (size_t j=0; j<3; ++j)
         {
              camera.R[i][j] = rgbInfo->r[i*3+j];
              camera.P[i][j] = rgbInfo->p[i*4+j];
         }
         camera.P[i][3] = rgbInfo->p[i*4+3];
   }
}


void
InputStereoCameraROS2::copyInfo(const sensor_msgs::msg::CameraInfo::SharedPtr rgbInfo, rvCameraIntrinsic &camera)
{
    camera.pixelWidth = rgbInfo->width;
    camera.pixelHeight = rgbInfo->height;
    camera.pixelStride = rgbInfo->width;
    camera.principalPoint[0] = rgbInfo->k[2];
    camera.principalPoint[1] = rgbInfo->k[5];
    camera.focalLength[0] = rgbInfo->k[0];
    camera.focalLength[1] = rgbInfo->k[4];
   
    printf("distortion model: ");
    if (rgbInfo->distortion_model == "rational_polynomial")
    {
        printf("rational_polynomial\n");
        camera.distortionModel = rvDistortionModel::RationalModel8;
   
        for(size_t i = 0; i< 8; i++)
        {
            camera.distortion[i] = rgbInfo->d[i];
        }
    }
    else
        printf("error\n");
}


bool InputStereoCameraROS2::ReadCameraConfig( const std::string & filename, rvCameraParams & cameraParams )
{
   printf("***ZYM*** %s\n",filename.c_str());
   cv::FileStorage fs_read( filename.c_str(), cv::FileStorage::READ );

   if (!fs_read.isOpened())
      return false;

   cameraParams.cameraType = rvStereo;

   cv::Size imageSize;
   fs_read["Image_Size"] >> imageSize;
   printf("***ZYM*** width %d height %d\n", imageSize.width, imageSize.height);
   cv::Mat cameraIntrinsics0, distortion0;
   std::string distortionModel;
   fs_read["distortion_model"]>>distortionModel;
   fs_read["Camera_Matrix1"] >> cameraIntrinsics0;
   fs_read["Distortion_Coefficients1"] >> distortion0;
   getCameraSetting(distortionModel, cameraIntrinsics0, distortion0, imageSize, cameraParams.stereo.camera[0]);

   cv::Mat cameraIntrinsics1, distortion1;
   fs_read["Camera_Matrix2"] >> cameraIntrinsics1;
   fs_read["Distortion_Coefficients2"] >> distortion1;
   fs_read["distortion_model"]>>distortionModel;
   getCameraSetting(distortionModel, cameraIntrinsics1, distortion1, imageSize, cameraParams.stereo.camera[1]);

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

   cameraParams.stereoRect.camera[0].initialized = false;
   cameraParams.stereoRect.camera[0].pixelHeight = imageSize.height;
   cameraParams.stereoRect.camera[0].pixelWidth = imageSize.width;
   cameraParams.stereoRect.camera[1].initialized = false;
   cameraParams.stereoRect.camera[1].pixelHeight = imageSize.height;
   cameraParams.stereoRect.camera[1].pixelWidth = imageSize.width;

   return true;
}


void InputStereoCameraROS2::getCameraSetting(const std::string & distortionModel, const cv::Mat & intrinsics, const cv::Mat & distortion, const cv::Size & imageSize, rvCameraIntrinsic & cameraConfig)
{
   cameraConfig.focalLength[0] = (float32_t)intrinsics.at<double>( 0, 0 );
   cameraConfig.focalLength[1] = (float32_t)intrinsics.at<double>( 1, 1 );
   cameraConfig.principalPoint[0] = (float32_t)intrinsics.at<double>( 0, 2 );
   cameraConfig.principalPoint[1] = (float32_t)intrinsics.at<double>( 1, 2 );

   cameraConfig.pixelWidth = imageSize.width;
   cameraConfig.pixelHeight = imageSize.height;
   cameraConfig.pixelStride = imageSize.width;

   memset( cameraConfig.distortion, 0, sizeof( cameraConfig.distortion ) );
   if (distortionModel == "fisheye")
   {
	   cameraConfig.distortionModel = rvDistortionModel::FisheyeModel4;
   }
   else
   {
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
      }
   for (int i=0; i<distortion.cols; i++ )
      cameraConfig.distortion[i] = (float32_t)distortion.at<double>( i );
}
