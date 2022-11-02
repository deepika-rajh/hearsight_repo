/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "EurocReader.h"

void ReadMatrix( std::ifstream & file, float * matrix );


#include "opencv2/opencv.hpp"
#include <fstream>

void copyToMat( const rvPose6DRT & s, cv::Mat &d )
{
   d = cv::Mat::eye( 4, 4, CV_32FC1 );

   for( size_t i = 0; i < 3; i++ )
      for( size_t j = 0; j < 4; j++ )
         d.at<float>( i, j ) = s.matrix[i][j];
}

EurocReader::EurocReader( const std::string & imageListFile ):
   SlamDataReader("", "", "","")
{
   ReadImageList( imageListFile, imageList );
   ReadStereoCameraConfig( imageListFile, cameraConfig, imuCamera0 );
   ReadIMUSamples( imageListFile, imuSamples );
   curImageIndex = 0;
   curIMUIndex = 0;
}

EurocReader::~EurocReader()
{

}

bool EurocReader::SkipNextFrame()
{
   curImageIndex++;
   return true;
}

bool EurocReader::GetNextFrame( mvFrame & frame, std::vector<sensor_wheel> & wheelOdomSet, std::vector<imu_pack_dsp> & imuSampleSet, std::vector<sensor_hijack> & hijackSet, std::vector<StampedSystemCallback> & callbackSet, std::vector<rvPose6DRTWithTimestamp>& poses)
{
   //Read image from file
   if( curImageIndex >= imageList.size() )
   {
      return false;
   }
   cv::Mat image = cv::imread( imageList[curImageIndex].leftImageName, cv::IMREAD_UNCHANGED );
   printf( "%s\n", imageList[curImageIndex].leftImageName.c_str() );
   if( nullptr == image.data )
   {
      printf( "Cannot read image with name: %s\n", imageList[curImageIndex].leftImageName.c_str() );
      imageList.resize( curImageIndex );
      return false;
   }

   cv::Mat iImage;
   if( image.channels() != 1 )
   {
      cv::cvtColor( image, iImage, cv::COLOR_BGR2GRAY );
   }
   else
   {
      iImage = image;
   }
   memcpy( frame.cameraName, "test", 5 );

   if( frame.leftImage->height != iImage.rows
       || frame.leftImage->width != iImage.cols
       || frame.leftImage->pixels == NULL )
   {
      ReleaseMVImage( frame.leftImage );
      AllocateMvImage( frame.leftImage, iImage.cols, image.rows, iImage.step[0] );
   }
   memcpy( frame.leftImage->pixels, iImage.data, iImage.rows * iImage.step[0] );

   //Get the timestamp

   cv::Mat rightImage = cv::imread( imageList[curImageIndex].rightImageName, cv::IMREAD_UNCHANGED );
   if( rightImage.channels() != 1 )
   {
      cv::cvtColor( rightImage, iImage, cv::COLOR_BGR2GRAY );
   }
   else
   {
      iImage = rightImage;
   }
   if( frame.rightImage->height != rightImage.rows
       || frame.rightImage->width != rightImage.cols
       || frame.rightImage->pixels == NULL )
   {
      ReleaseMVImage( frame.rightImage );
      AllocateMvImage( frame.rightImage, iImage.cols, iImage.rows, iImage.step[0] );
   }
   memcpy( frame.rightImage->pixels, rightImage.data, iImage.step[0] * iImage.rows);
   frame.timestamp = imageList[curImageIndex].timestamp;
   
   curImageIndex++;


   imuSampleSet.clear();
   while ( curIMUIndex < imuSamples.size() && imuSamples[curIMUIndex].time_acc < frame.timestamp + 2e7  )
   {
      imuSampleSet.push_back( imuSamples[curIMUIndex] );
      curIMUIndex++;
   } 

   return true;

}

void EurocReader::ReadImageList( const std::string & dataPath, std::vector<StereoSample> & imageNameSet )
{
   std::vector<fileWithTimestamp> leftSamples, rightSamples;
   LoadFileSamples( dataPath + "/mav0/cam0/data.csv", leftSamples );
   LoadFileSamples( dataPath + "/mav0/cam1/data.csv", rightSamples );


   struct TimestampPair
   {
      size_t rgbIndex;
      size_t depthIndex;
      int64_t diff;
   };


   struct
   {
      bool operator()( const TimestampPair& a, const TimestampPair& b ) const
      {
         return a.diff < b.diff;
      }
      bool operator()( const StereoSample& a, const StereoSample& b ) const
      {
         return a.timestamp < b.timestamp;
      }
   } customLess;


   std::vector<TimestampPair> timestampPairSet;
   TimestampPair timePair;
   timestampPairSet.reserve( 10000 );
   int64_t invalidDif = 2000000;
   size_t rgbIndex = 0, depthIndex = 0, depthLower = 0, depthUpper = rightSamples.size();
   for( rgbIndex = 0; rgbIndex < leftSamples.size(); rgbIndex++ )
      for( depthIndex = depthLower; depthIndex < depthUpper; depthIndex++ )
      {
         if( leftSamples[rgbIndex].timestamp - invalidDif > rightSamples[depthIndex].timestamp )
         {
            depthLower = depthIndex;
            continue;
         }
         if( leftSamples[rgbIndex].timestamp + invalidDif < rightSamples[depthIndex].timestamp )
         {
            break;
         }
         timePair.rgbIndex = rgbIndex;
         timePair.depthIndex = depthIndex;
         timePair.diff = abs( leftSamples[rgbIndex].timestamp - rightSamples[depthIndex].timestamp );
         timestampPairSet.push_back( timePair );
      }
   std::sort( timestampPairSet.begin(), timestampPairSet.end(), customLess );


   StereoSample sample;
   imageNameSet.reserve( timestampPairSet.size() );
   for( rgbIndex = 0; rgbIndex < timestampPairSet.size(); rgbIndex++ )
   {
      if( leftSamples[timestampPairSet[rgbIndex].rgbIndex].timestamp < 0 )
         continue;

      if( rightSamples[timestampPairSet[rgbIndex].depthIndex].timestamp < 0 )
         continue;

      sample.timestamp = leftSamples[timestampPairSet[rgbIndex].rgbIndex].timestamp;
      sample.leftImageName = dataPath + "/mav0/cam0/data/" + leftSamples[timestampPairSet[rgbIndex].rgbIndex].filename;
      sample.rightImageName = dataPath + "/mav0/cam1/data/" + rightSamples[timestampPairSet[rgbIndex].depthIndex].filename;
      leftSamples[timestampPairSet[rgbIndex].rgbIndex].timestamp = -1;
      rightSamples[timestampPairSet[rgbIndex].depthIndex].timestamp = -1;

      imageNameSet.push_back( sample );
   };
   std::sort( imageNameSet.begin(), imageNameSet.end(), customLess );
}


void EurocReader::LoadFileSamples( const std::string& filename, std::vector<fileWithTimestamp>& sampleSet )
{
   std::ifstream f;
   f.open( filename.c_str() );
   fileWithTimestamp sample;
   sampleSet.reserve( 10000 );
   char tmp;
   while( !f.eof() )
   {
      std::string s;
      std::getline( f, s );
      if( !s.empty() && s[0] != '#' )
      {
         std::stringstream ss;
         ss << s;
         ss >> sample.timestamp;
         ss >> tmp;
         ss >> sample.filename;
         sampleSet.push_back( sample );
      }
   }
}

void EurocReader::ReadStereoCameraConfig( const std::string & imageListFile, rvStereoCamera & cameraConfig, rvPose6DRT & imuCamera0 )
{
   rvPose6DRT imuCamera1;
   ReadCameraConfig( imageListFile + "/mav0/cam0/sensor.yaml", cameraConfig.camera[0], imuCamera0 );
   ReadCameraConfig( imageListFile + "/mav0/cam1/sensor.yaml", cameraConfig.camera[1], imuCamera1 );
   cv::Mat imuC0( 4, 4, CV_32FC1 ), imuC1( 4, 4, CV_32FC1 ), c1C0( 4, 4, CV_32FC1 );
   copyToMat( imuCamera0, imuC0 );
   copyToMat( imuCamera1, imuC1 );
   c1C0 = imuC1.inv() * imuC0;
   cv::Mat vec;
   cv::Mat tmp = c1C0( cv::Range( 0, 3 ), cv::Range( 0, 3 ) );
   cv::Rodrigues( tmp, vec );
   for( size_t i = 0; i < 3; i++ )
   {
      cameraConfig.translation[i] = c1C0.at<float>( i, 3 );
      cameraConfig.rotation[i] = vec.at<float>( i );
   }
}

void EurocReader::ReadCameraConfig( const std::string & filename, rvCameraIntrinsic & cameraConfig, rvPose6DRT & bodyCamera )
{
   std::ifstream f;
   f.open( filename.c_str() );   
   char tmp;
   while( !f.eof() )
   {
      std::string s;
      std::getline( f, s );
      if( !s.empty() && s[0] != '#' )
      {
         if( s.compare( "T_BS:" ) == 0 )
         {
            float matrix[16];
            ReadMatrix( f, matrix );
            for (size_t i=0,k=0; i<3; i++ )
               for( size_t j = 0; j < 4; j++,k++ )
               {
                  bodyCamera.matrix[i][j] = matrix[k];
               }
         }
         else
         {
            std::stringstream ss;
            ss << s;
            std::string keyword;
            ss >> keyword;
            if( keyword.compare( "resolution:" ) == 0 )
            {
               ss >> tmp >> cameraConfig.pixelWidth;
               ss >> tmp >> cameraConfig.pixelHeight;
            }
            else if( keyword.compare( "intrinsics:" ) == 0 )
            {
               ss >> tmp >> cameraConfig.focalLength[0];
               ss >> tmp >> cameraConfig.focalLength[1];
               ss >> tmp >> cameraConfig.principalPoint[0];
               ss >> tmp >> cameraConfig.principalPoint[1];
            }
            else if( keyword.compare( "distortion_coefficients:" ) == 0 )
            {
               memset( cameraConfig.distortion, 0, sizeof( cameraConfig.distortion ) );
               cameraConfig.distortionModel = Polynomial4;
               ss >> tmp >> cameraConfig.distortion[0];
               ss >> tmp >> cameraConfig.distortion[1];
               ss >> tmp >> cameraConfig.distortion[2];
               ss >> tmp >> cameraConfig.distortion[3];
            }
         }
      }
   }
}

void EurocReader::ReadIMUSamples( const std::string & imageListFile, std::vector<imu_pack_dsp> & imuSampleSet )
{
   std::string fileName = imageListFile + "/mav0/imu0/data.csv";
   std::ifstream f;
   f.open( fileName.c_str() );
   imu_pack_dsp sample;
   imuSampleSet.reserve( 10000 );
   char tmp;
   while( !f.eof() )
   {
      std::string s;
      std::getline( f, s );
      if( !s.empty() && s[0] != '#' )
      {
         std::stringstream ss;
         ss << s;
         ss >> sample.time_acc;
         sample.time_gyro = sample.time_acc;
         ss >> tmp >> sample.angular_velocity_x;
         ss >> tmp >> sample.angular_velocity_y;
         ss >> tmp >> sample.angular_velocity_z;
         ss >> tmp >> sample.acceloration_x;
         ss >> tmp >> sample.acceloration_y;
         ss >> tmp >> sample.acceloration_z;
         imuSampleSet.push_back( sample );
      }
   }
}

bool EurocReader::getCameraConfiguration( rvCameraParams & config )
{
   config.imageFormat = YUV_FORMAT;
   config.cameraType = rvStereo;
   config.stereo = cameraConfig;
   memcpy( config.stereoRect.translation, cameraConfig.translation, sizeof( cameraConfig.translation ) );
 
   double R0[3][3] = { 0.999966347530033, -0.001422739138722922, 0.008079580483432283, 
      0.001365741834644127, 0.9999741760894847, 0.007055629199258132, 
      -0.008089410156878961, -0.007044357138835809, 0.9999424675829176 };

   double P0[3][4] = { 435.2046959714599, 0, 367.4517211914062, 0,  
      0, 435.2046959714599, 252.2008514404297, 0, 
      0, 0, 1, 0 };

   double R1[3][3] = { 0.9999633526194376, -0.003625811871560086, 0.007755443660172947, 
      0.003680398547259526, 0.9999684752771629, -0.007035845251224894, 
      -0.007729688520722713, 0.007064130529506649, 0.999945173484644 };

   double P1[3][4] = { 435.2046959714599, 0, 367.4517211914062, -47.90639384423901, 
      0, 435.2046959714599, 252.2008514404297, 0, 
      0, 0, 1, 0 };

   for( size_t i = 0; i < 3; i++ )
   {
      for( size_t j = 0; j < 3; j++ )
      {
         config.stereoRect.camera[0].P[i][j] = P0[i][j];
         config.stereoRect.camera[0].R[i][j] = R0[i][j];
         config.stereoRect.camera[1].P[i][j] = P0[i][j];
         config.stereoRect.camera[1].R[i][j] = R1[i][j];
      }
      config.stereoRect.camera[0].P[i][3] = P0[i][3];
      config.stereoRect.camera[1].P[i][3] = P1[i][3];
   }

   config.stereoRect.camera[0].initialized = true;
   config.stereoRect.camera[1].initialized = true;

   config.stereoRect.camera[0].pixelHeight = 480;
   config.stereoRect.camera[0].pixelWidth = 752;
   config.stereoRect.camera[1].pixelHeight = 480;
   config.stereoRect.camera[1].pixelWidth = 752;

   return true;
}


bool EurocReader::getIMUConfiguration( rvIMUConfiguration & config )
{
   config.acceBias[0] = 0.f;
   config.acceBias[1] = 0.f;
   config.acceBias[2] = 0.f;

   config.cameraInIMU = imuCamera0;

   config.deltaInSecond = 0.f;

   config.gyroBias[0] = 0.f;
   config.gyroBias[1] = 0.f;
   config.gyroBias[2] = 0.f;

   config.imuEnabled = true;

   return true;
}


bool EurocReader::getWheelConfiguration( rvWheelConfiguration & config )
{
   config.wheelEnabled = false;
   for( size_t i = 0; i < 3; i++ )
   {
      for( size_t j = 0; j < 4; j++ )
         config.baselinkInCamera.matrix[i][j] = 0.f;
      config.baselinkInCamera.matrix[i][i] = 1.f;
   }

   return true;
}

