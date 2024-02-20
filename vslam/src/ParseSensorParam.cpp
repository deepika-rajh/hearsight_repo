/*******************************************************************************
@copyright
Copyright (c) 2022-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "ParseSensorParam.h"
#include <fstream>
#include <sstream>
#include "memory.h"
#include "math.h"

#include <opencv2/opencv.hpp>
void getCameraSetting(const cv::Mat& intrinsics, const std::string& distortionModelName, const cv::Mat& distortion, const cv::Size& imageSize, rvCameraIntrinsic& cameraConfig);
void initRectificationFisheye(const rvStereoCamera& stereoCamera, cv::Mat& R0, cv::Mat& P0, cv::Mat& R1, cv::Mat& P1, int& width, int& height);

void EulerToSO3_0( const float32_t* euler, float32_t* rotation );
bool ReadIMUParamters( const char * imuFile, rvIMUConfiguration & imuParameter );

bool ParseSensorParam( const std::string & root, const std::string & configFile, rvWheelConfiguration & wheelConf, rvIMUConfiguration & imuConf, rvTargetImage &targetImage )
{
   bool crossT = false, crossR = false;
   std::ifstream cfg( root+configFile, std::ifstream::in );
   std::string tempRoot = root;

   if( !cfg.is_open() )
   {
      printf( "Fail to open configuration file: %s\n", configFile.c_str() );
      cfg.open( root + "../" + configFile, std::ifstream::in );
      if( !cfg.is_open() )
      {
         printf( "Fail to open configuration file: %s also.\n", ("../" + configFile).c_str() );
         return false;
      }
      tempRoot = root + "../";
   }

   std::string line;
   std::string itemName;
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
      itemName.clear();
      iss >> itemName;
      if( itemName.compare( "WEF.Tvb" ) == 0 )
      {
         printf("WEF.Tvb****\n");
         float translation[3];
         iss >> translation[0] >> translation[1] >> translation[2];
         wheelConf.baselinkInCamera.matrix[0][3] = translation[0];
         wheelConf.baselinkInCamera.matrix[1][3] = translation[1];
         wheelConf.baselinkInCamera.matrix[2][3] = translation[2]; 
         crossT = true;
      }
      else if( itemName.compare( "WEF.Rvb" ) == 0 )
      {
         float euler[3];
         iss >> euler[0] >> euler[1] >> euler[2];
         //https://en.wikipedia.org/wiki/Euler_angles#Tait%E2%80%93Bryan_angles
         //Section Conversion to other orientation representations->Rotation matrix
         //This euler angle is defined as Z1Y2X3 according to the conversion table in the section mentioned above
         //Which is different from the defintion of mvPose6DET in mv.h
         float rotation[9];
         EulerToSO3_0( euler, rotation );
         memcpy( wheelConf.baselinkInCamera.matrix[0], rotation + 0, sizeof( float ) * 3 );
         memcpy( wheelConf.baselinkInCamera.matrix[1], rotation + 3, sizeof( float ) * 3 );
         memcpy( wheelConf.baselinkInCamera.matrix[2], rotation + 6, sizeof( float ) * 3 );
         crossR = true;
      }
      else if( itemName.compare( "IMU" ) == 0 )
      {
         std::string imuFile;
         iss >> imuFile;
         imuFile = root + imuFile;
         printf("root File path is: %s ,imu file path is: %s\n", root.c_str(), imuFile.c_str());
         ReadIMUParamters( imuFile.c_str(), imuConf );
      }
      else if (itemName.compare("footprintSize") == 0)
      {
          iss >> wheelConf.footprintSize;
      }
      else if (itemName.compare("TargetImage") == 0)
      {
          std::string path;
          iss >> path;
          targetImage.path = tempRoot + path;
      }
      else if (itemName.compare("TargetWidth") == 0)
      {
          iss >> targetImage.targetWidth;
      }
      else if (itemName.compare("TargetHeight") == 0)
      {
          iss >> targetImage.targetHeight;
      }
   }
   if( crossR && crossT )
   {
      wheelConf.wheelEnabled = true;
   }
   return true;
}


bool GetCameraSettingFile( const std::string & root, const std::string & configFile, std::string & cameraSettingFile)
{
   std::ifstream cfg( root+configFile, std::ifstream::in );
   std::string tempRoot = root;

   if( !cfg.is_open() )
   {
      printf( "Fail to open configuration file: %s\n", configFile.c_str() );
      cfg.open( root + "../" + configFile, std::ifstream::in );
      if( !cfg.is_open() )
      {
         printf( "Fail to open configuration file: %s also.\n", ("../" + configFile).c_str() );
         return false;
      }
      tempRoot = root + "../";
   }

   std::string line;
   std::string itemName;
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
      itemName.clear();
      iss >> itemName;
      if( itemName.compare( "Camera" ) == 0 )
      {
         iss >> cameraSettingFile;
      }
      else if( itemName.compare( "Stereo" ) == 0 )
      {
         iss >> cameraSettingFile;
      }
  }
  return true;
}

void EulerToSO3_0( const float32_t* euler, float32_t* rotation )
{
   float32_t cr = (float32_t)cos( euler[0] );
   float32_t sr = (float32_t)sin( euler[0] );
   float32_t cp = (float32_t)cos( euler[1] );
   float32_t sp = (float32_t)sin( euler[1] );
   float32_t cy = (float32_t)cos( euler[2] );
   float32_t sy = (float32_t)sin( euler[2] );
   rotation[0 * 3 + 0] = cy*cp;
   rotation[0 * 3 + 1] = cy*sp*sr - sy*cr;
   rotation[0 * 3 + 2] = cy*sp*cr + sy*sr;
   rotation[1 * 3 + 0] = sy*cp;
   rotation[1 * 3 + 1] = sy*sp*sr + cy*cr;
   rotation[1 * 3 + 2] = sy*sp*cr - cy*sr;
   rotation[2 * 3 + 0] = -sp;
   rotation[2 * 3 + 1] = cp*sr;
   rotation[2 * 3 + 2] = cp*cr;
}

bool ReadIMUParamters( const char * imuFile, rvIMUConfiguration & imuParameter )
{
   std::string fullName = imuFile;
   std::ifstream cfg( fullName, std::ifstream::in );
   if( !cfg.is_open() )
   {
      printf( "Fail to open imu configuration file: %s\n", fullName.c_str() );
      return false;
   }

   std::string line;
   std::string itemName;
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
      if( itemName.compare( "delta:" ) == 0 )
      {
         iss >> imuParameter.deltaInSecond;
      }
      else if( itemName.compare( "Accelerator_bias:" ) == 0 )
      {
         ReadMatrix( cfg, imuParameter.acc.bias );
      }
      else if( itemName.compare( "Gyro_bias:" ) == 0 )
      {
         ReadMatrix( cfg, imuParameter.gyr.bias );
      }
      else if( itemName.compare( "Camera_in_IMU:" ) == 0 )
      {
         float p[12];
         ReadMatrix( cfg, p );
         for( size_t i = 0, k = 0; i < 3; i++ )
            for( size_t j = 0; j < 4; j++, k++ )
            {
               imuParameter.cameraInIMU.matrix[i][j] = p[k];
            }
      }
   }

   return true;
}


void ReadMatrix( std::ifstream & file, float * matrix )
{
   std::string line, valName;
   int rows=0, cols=0;

   std::getline( file, line );
   std::istringstream issRow ( line );
   issRow >> valName >> rows;

   std::getline( file, line );
   std::istringstream issCol( line );
   issCol >> valName >> cols;

   std::getline( file, line );
   if (std::string::npos == line.find_first_of('['))
       std::getline(file, line);
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

bool readStereoCameraParameter(const char* cameraID, rvStereoCamera& configuration, rvStereoRectCamera& outputCamera)
{
   printf( "***ZYM*** %s\n", cameraID );
   cv::FileStorage fs_read( cameraID, cv::FileStorage::READ );

   std::string distortionModelName;
   fs_read["distortion_model"] >> distortionModelName;

   //cameras->type = mvStereo;

   cv::Size imageSize;
   fs_read["Image_Size"] >> imageSize;
   printf( "***ZYM*** width %d height %d\n", imageSize.width, imageSize.height );
   cv::Mat cameraIntrinsics0, distortion0;
   fs_read["Camera_Matrix1"] >> cameraIntrinsics0;
   fs_read["Distortion_Coefficients1"] >> distortion0;
   getCameraSetting( cameraIntrinsics0, distortionModelName, distortion0, imageSize, configuration.camera[0] );

   cv::Mat cameraIntrinsics1, distortion1;
   fs_read["Camera_Matrix2"] >> cameraIntrinsics1;
   fs_read["Distortion_Coefficients2"] >> distortion1;
   getCameraSetting( cameraIntrinsics1, distortionModelName, distortion1, imageSize, configuration.camera[1] );

   cv::Mat rotation, r;
   fs_read["R"] >> rotation;

   cv::Mat t;
   fs_read["T"] >> t;
   t = t / 1000.;

   if( t.at<double>( 0 ) > 0 )
   {
      rotation = rotation.inv();
      t = -rotation * t;
   }

   cv::Rodrigues( rotation, r );
   configuration.rotation[0] = (float32_t)r.at<double>( 0 );
   configuration.rotation[1] = (float32_t)r.at<double>( 1 );
   configuration.rotation[2] = (float32_t)r.at<double>( 2 );

   configuration.translation[0] = (float32_t)t.at<double>( 0 );
   configuration.translation[1] = (float32_t)t.at<double>( 1 );
   configuration.translation[2] = (float32_t)t.at<double>( 2 );

    outputCamera.camera[0].initialized = true;
    outputCamera.camera[1].initialized = true;

    cv::Mat R0, R1, P0, P1, Q;
    bool usingOpenCVRectiyFun = false;
    if (usingOpenCVRectiyFun)
    {
        if (distortionModelName.compare("fisheye") == 0)
        {
            cv::fisheye::stereoRectify(cameraIntrinsics0, distortion0, cameraIntrinsics1, distortion1,
                imageSize, r, t, R0, R1, P0, P1, Q, cv::CALIB_ZERO_DISPARITY);
        }
        else
        {
            cv::stereoRectify(cameraIntrinsics0, distortion0, cameraIntrinsics1, distortion1,
                imageSize, r, t, R0, R1, P0, P1, Q, cv::CALIB_ZERO_DISPARITY, 0);
        }
        outputCamera.camera[0].pixelHeight = imageSize.height;
        outputCamera.camera[0].pixelWidth = imageSize.width;
        outputCamera.camera[1].pixelHeight = imageSize.height;
        outputCamera.camera[1].pixelWidth = imageSize.width;
    }
    else
    {
        int imgW, imgH;
        initRectificationFisheye(configuration, R0, P0, R1, P1, imgW, imgH);
        outputCamera.camera[0].pixelHeight = imgH;
        outputCamera.camera[0].pixelWidth = imgW;
        outputCamera.camera[1].pixelHeight = imgH;
        outputCamera.camera[1].pixelWidth = imgW;
    }

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            outputCamera.camera[0].P[i][j] = P0.at<double>(i, j);
            outputCamera.camera[1].P[i][j] = P1.at<double>(i, j);
            outputCamera.camera[0].R[i][j] = R0.at<double>(i, j);
            outputCamera.camera[1].R[i][j] = R1.at<double>(i, j);
        }
        outputCamera.camera[0].P[i][3] = P0.at<double>(i, 3);
        outputCamera.camera[1].P[i][3] = P1.at<double>(i, 3);
    }
    outputCamera.translation[0] = (float32_t)(outputCamera.camera[1].P[0][3] / outputCamera.camera[1].P[0][0]);
    outputCamera.translation[1] = outputCamera.translation[2] = 0;
    return true;
}

void getCameraSetting( const cv::Mat & intrinsics, const std::string& distortionModelName, const cv::Mat & distortion, const cv::Size & imageSize, rvCameraIntrinsic & cameraConfig )
{
   cameraConfig.focalLength[0] = (float32_t)intrinsics.at<double>( 0, 0 );
   cameraConfig.focalLength[1] = (float32_t)intrinsics.at<double>( 1, 1 );
   cameraConfig.principalPoint[0] = (float32_t)intrinsics.at<double>( 0, 2 );
   cameraConfig.principalPoint[1] = (float32_t)intrinsics.at<double>( 1, 2 );

   cameraConfig.pixelWidth = imageSize.width;
   cameraConfig.pixelHeight = imageSize.height;
   cameraConfig.pixelStride = cameraConfig.pixelWidth;

    memset(cameraConfig.distortion, 0, sizeof(cameraConfig.distortion));
    if (distortionModelName.compare("fisheye") == 0)
    {
        cameraConfig.distortionModel = rvDistortionModel::FisheyeModel4;
    }
    else
    {
        switch (distortion.cols)
        {
        case 8:
            cameraConfig.distortionModel = rvDistortionModel::RationalModel8;
            break;
        case 5:
            cameraConfig.distortionModel = rvDistortionModel::Polynomial5;
            break;
        case 4:
            cameraConfig.distortionModel = rvDistortionModel::Polynomial4;
            break;
        default:
            cameraConfig.distortionModel = rvDistortionModel::NoDistortion;
        }
    }
    for (int i = 0; i < distortion.cols; i++)
        cameraConfig.distortion[i] = (float32_t)distortion.at<double>(i);
}


void initRectificationFisheye(const rvStereoCamera& stereoCamera, cv::Mat& R0, cv::Mat& P0, cv::Mat& R1, cv::Mat& P1, int& width, int& height)
{

    //https://github.com/IntelRealSense/librealsense/pull/3951/files
    //We need to determine what focal length our undistorted images should have
    //in order to set up the camera matrices for initUndistortRectifyMap.We
    //could use stereoRectify, but here we show how to derive these projection
    //matrices from the calibration and a desired height and field of view

    //We calculate the undistorted focal length :
    //
    //         h
    // -----------------
    //  \      |      /
    //    \    | f  /
    //     \   |   /
    //      \ fov /
    //        \|/
    float stereo_fov_rad = 100 * (3.14159f / 180);  // 100 degree desired fov
    int stereo_height_px = 360;                   // 300x300 pixel stereo output
    float stereo_focal_px = stereo_height_px / 2 / tan(stereo_fov_rad / 2);

    //We set the left rotation to identity and the right rotation
    //the rotation between the cameras
    //cv::Mat  R_left = cv::Mat::eye( 3, 3, CV_32FC1 );
    cv::Mat  R_right, R_left;
    cv::Mat T1(3, 1, CV_64FC1), T0;
    cv::Mat r(3, 1, CV_64FC1);
    for (size_t i = 0; i < 3; i++)
    {
        T1.at<double>(i) = (double)stereoCamera.translation[i];
        r.at<double>(i) = (double)stereoCamera.rotation[i];
    }

    cv::Rodrigues(-r / 2, R_right);  //if right's attitude is identity, left's attitude is r. if left is idenetiy, right's is -r.
    R_left = R_right.t();

    T0 = R_right * T1;
    double normT = cv::norm(T0);
    T0 /= normT;
    cv::Mat ep1 = cv::Mat::zeros(3, 1, CV_64FC1);
    ep1.at<double>(0) = 1;
    if (ep1.dot(T0) < 0)
        ep1 = -ep1;
    cv::Mat ep2 = T0.cross(ep1);
    ep2 /= cv::norm(ep2);
    ep2 *= acos(fabs(T0.dot(ep1)));
    cv::Mat R2;
    cv::Rodrigues(ep2, R2);
    R0 = R2 * R_left;
    R1 = R2 * R_right;
    T1 = R_right * T1;


    //The stereo algorithm needs max_disp extra pixels in order to produce valid
    //disparity on the desired output region.This changes the width, but the
    //center of projection should be on the center of the cropped image
    float max_disp = 280; //16*8
    int stereo_width_px = stereo_height_px + max_disp;
    float stereo_cx = (stereo_width_px - 1) / 2.0f;
    float stereo_cy = (stereo_height_px - 1) / 2.0f;

    P0 = cv::Mat::zeros(3, 4, CV_64FC1);
    P0.at<double>(0, 0) = stereo_focal_px;
    P0.at<double>(1, 1) = stereo_focal_px;
    P0.at<double>(2, 2) = 1.;
    P0.at<double>(0, 2) = stereo_cx;
    P0.at<double>(1, 2) = stereo_cy;

    P1 = P0.clone();
    P1.at<double>(0, 3) = T1.at<double>(0) * stereo_focal_px;
    width = stereo_width_px;
    height = stereo_height_px;

    return;
}
