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
#include "opencv2/opencv.hpp"

#include "mv.h"
#include "rvCamera.h"

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
      }else if( itemName.compare( "IMU" ) == 0 )
      {
         std::string imuFile;
         iss >> imuFile;
         imuFile = root + imuFile;
		 printf("root File path is: %s ,imu file path is: %s\n", root.c_str(), imuFile.c_str());
         ReadIMUParamters( imuFile.c_str(), imuConf );
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
         ReadMatrix( cfg, imuParameter.acceBias );
      }
      else if( itemName.compare( "Gyro_bias:" ) == 0 )
      {
         ReadMatrix( cfg, imuParameter.gyroBias );
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

void getCameraSetting( const cv::Mat & intrinsics, const std::string& distortionModelName, const cv::Mat & distortion, const cv::Size & imageSize, rvCameraIntrinsic & cameraConfig );

bool readStereoCameraParameter( const char *cameraID, rvStereoCamera & configuration, rvStereoRectCamera & outputCamera )
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

   outputCamera.camera[0].initialized = false;
   outputCamera.camera[0].pixelHeight = imageSize.height;
   outputCamera.camera[0].pixelWidth = imageSize.width;
   outputCamera.camera[1].initialized = false;
   outputCamera.camera[1].pixelHeight = imageSize.height;
   outputCamera.camera[1].pixelWidth = imageSize.width;

   //cv::Mat R0, R1, P0, P1, Q;
   //if (distortionModelName.compare("fisheye") == 0)
   //{
   //   cv::fisheye::stereoRectify(cameraIntrinsics0, distortion0, cameraIntrinsics1, distortion1,
   //      imageSize, r, t, R0, R1, P0, P1, Q, cv::CALIB_ZERO_DISPARITY);
   //}
   //else
   //{
   //   cv::stereoRectify( cameraIntrinsics0, distortion0, cameraIntrinsics1, distortion1,
   //                   imageSize, r, t, R0, R1, P0, P1, Q, cv::CALIB_ZERO_DISPARITY, 0 );
   //}

   //for( size_t i = 0; i < 3; i++ )
   //{
   //   for( size_t j = 0; j < 3; j++ )
   //   {
   //      outputCamera.camera[0].P[i][j] = P0.at<double>( i, j );
   //      outputCamera.camera[1].P[i][j] = P1.at<double>( i, j );
   //      outputCamera.camera[0].R[i][j] = R0.at<double>( i, j );
   //      outputCamera.camera[1].R[i][j] = R1.at<double>( i, j );
   //   }
   //   outputCamera.camera[0].P[i][3] = P0.at<double>( i, 3 );
   //   outputCamera.camera[1].P[i][3] = P1.at<double>( i, 3 );
   //}
   //outputCamera.translation[0] = (float32_t) (outputCamera.camera[1].P[0][3] / outputCamera.camera[1].P[0][0]);
   //outputCamera.translation[1] = outputCamera.translation[2] = 0;

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

   memset( cameraConfig.distortion, 0, sizeof( cameraConfig.distortion ) );   
   if (distortionModelName.compare("fisheye") == 0)
   {
      cameraConfig.distortionModel = rvDistortionModel::FisheyeModel4;
   }
   else
   {
       switch (distortion.cols)
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
