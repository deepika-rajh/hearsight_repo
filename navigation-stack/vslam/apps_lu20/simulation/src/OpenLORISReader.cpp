/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "OpenLORISReader.h"

void ReadMatrix( std::ifstream & file, float * matrix );


#include "opencv2/opencv.hpp"
#include <fstream>

void copyToMat( const rvPose6DRT & s, cv::Mat &d );

OpenLORISReader::OpenLORISReader( const std::string & imageListFile ):
   SlamDataReader("", "", "","")
{
   curImageIndex = 0;
   curIMUIndex = 0;
   curWheelIndex = 0;
}

OpenLORISReader::~OpenLORISReader()
{

}

bool OpenLORISReader::SkipNextFrame()
{
   if( curImageIndex >= imageList.size() )
   {
      return false;
   }

   uint64_t timestamp = imageList[curImageIndex].timestamp;
   curImageIndex++;

   while( imuSamples[curIMUIndex].time_acc < timestamp + 2e7 )
   {
      curIMUIndex++;
   }

   while( wheelSamples[curWheelIndex].timestamp < timestamp + 2e7 )
   {
      curWheelIndex++;
   }
   return true;
}

void OpenLORISReader::ReadImageList( const std::string & dataPath, const std::string & leftList, const std::string & rightList, std::vector<RGBDSample> & imageNameSet )
{
   std::vector<fileWithTimestamp> imageSamples, depthSamples;
   LoadFileSamples( dataPath + "/" + leftList, imageSamples );
   LoadFileSamples( dataPath + "/" + rightList, depthSamples );


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
      bool operator()( const RGBDSample& a, const RGBDSample& b ) const
      {
         return a.timestamp < b.timestamp;
      }
   } customLess;


   std::vector<TimestampPair> timestampPairSet;
   TimestampPair timePair;
   timestampPairSet.reserve( 10000 );
   int64_t invalidDif = 10000000;
   size_t rgbIndex = 0, depthIndex = 0, depthLower = 0, depthUpper = depthSamples.size();
   for( rgbIndex = 0; rgbIndex < imageSamples.size(); rgbIndex++ )
      for( depthIndex = depthLower; depthIndex < depthUpper; depthIndex++ )
      {
         if( imageSamples[rgbIndex].timestamp - invalidDif > depthSamples[depthIndex].timestamp )
         {
            depthLower = depthIndex;
            continue;
         }
         if( imageSamples[rgbIndex].timestamp + invalidDif < depthSamples[depthIndex].timestamp )
         {
            break;
         }
         timePair.rgbIndex = rgbIndex;
         timePair.depthIndex = depthIndex;
         timePair.diff = abs( imageSamples[rgbIndex].timestamp - depthSamples[depthIndex].timestamp );
         timestampPairSet.push_back( timePair );
      }
   std::sort( timestampPairSet.begin(), timestampPairSet.end(), customLess );


   RGBDSample sample;
   imageNameSet.reserve( timestampPairSet.size() );
   for( rgbIndex = 0; rgbIndex < timestampPairSet.size(); rgbIndex++ )
   {
      if( imageSamples[timestampPairSet[rgbIndex].rgbIndex].timestamp < 0 )
         continue;

      if( depthSamples[timestampPairSet[rgbIndex].depthIndex].timestamp < 0 )
         continue;

      sample.timestamp = imageSamples[timestampPairSet[rgbIndex].rgbIndex].timestamp;
      sample.grayImageName = dataPath + imageSamples[timestampPairSet[rgbIndex].rgbIndex].filename;
      sample.depthImageName = dataPath + depthSamples[timestampPairSet[rgbIndex].depthIndex].filename;
      imageSamples[timestampPairSet[rgbIndex].rgbIndex].timestamp = -1;
      depthSamples[timestampPairSet[rgbIndex].depthIndex].timestamp = -1;

      imageNameSet.push_back( sample );
   };
   std::sort( imageNameSet.begin(), imageNameSet.end(), customLess );
}


void OpenLORISReader::LoadFileSamples( const std::string& filename, std::vector<fileWithTimestamp>& sampleSet )
{
   std::ifstream f;
   f.open( filename.c_str() );
   fileWithTimestamp sample;
   sampleSet.reserve( 10000 );
   double stampInSecond;
   while( !f.eof() )
   {
      std::string s;
      std::getline( f, s );
      if( !s.empty() && s[0] != '#' )
      {
         std::stringstream ss;
         ss << s;
         ss >> stampInSecond;
         sample.timestamp = (int64_t)(stampInSecond*1e9);
         ss >> sample.filename;
         sampleSet.push_back( sample );
      }
   }
}


void OpenLORISReader::ReadMonoCameraConfig( const std::string & filename, const std::string & cameraName, rvCameraIntrinsic & cameraConfig )
{
   cv::FileStorage fs_read( filename.c_str(), cv::FileStorage::READ );
   int height;
   fs_read[cameraName]["height"] >> height; // 
   cameraConfig.pixelHeight = (uint32_t) height;
   fs_read[cameraName]["width"] >> height;
   cameraConfig.pixelWidth = (uint32_t)height;
   std::string distortionModel;
   fs_read[cameraName]["distortion_model"] >> distortionModel;
   cv::Mat distortionCoeff;
   fs_read[cameraName]["distortion_coefficients"] >> distortionCoeff;

   memset( cameraConfig.distortion, 0, sizeof( cameraConfig.distortion ) );
   if (cv::norm(distortionCoeff) ==0 )
      cameraConfig.distortionModel = rvDistortionModel::NoDistortion;
   else if( distortionModel.compare( "Kannala-Brandt" ) == 0 )
   {
      cameraConfig.distortionModel = rvDistortionModel::FisheyeModel4;
      cameraConfig.distortion[0] = (float32_t)distortionCoeff.at<double>( 0 );
      cameraConfig.distortion[1] = (float32_t)distortionCoeff.at<double>( 1 );
      cameraConfig.distortion[2] = (float32_t)distortionCoeff.at<double>( 2 );
      cameraConfig.distortion[3] = (float32_t)distortionCoeff.at<double>( 3 );
   }
   else
   {
      assert( 0 );
   }
   cv::Mat cameraIntrinsics;
   fs_read[cameraName]["intrinsics"] >> cameraIntrinsics;
   cameraConfig.focalLength[0] = (float32_t)cameraIntrinsics.at<double>( 0 );
   cameraConfig.focalLength[1] = (float32_t)cameraIntrinsics.at<double>( 2 );
   cameraConfig.principalPoint[0] = (float32_t)cameraIntrinsics.at<double>( 1 );
   cameraConfig.principalPoint[1] = (float32_t)cameraIntrinsics.at<double>( 3 );
}


void OpenLORISReader::ReadExtrinsicParams( const std::string & transFile,
                                           const std::string & cameraFrameL,
                                           const std::string & cameraFrameR,
                                           const std::string & imuFrame,
                                           rvPose6DRT & imuCamera,
                                           rvPose6DRT & imuWheel,
                                           rvPose6DRT & stereoCameraExtrinsic )
{
   cv::FileStorage fs_read( transFile.c_str(), cv::FileStorage::READ );
   cv::FileNode matrixes = fs_read["trans_matrix"];
   if( matrixes.type() != cv::FileNode::SEQ )
   {
      std::cerr << "strings is not a sequence! FAIL" << std::endl;
   }

   std::string parent, child;
   cv::Mat baselinkLaser, laserCamera;
   cv::Mat cameraExtrinsic;
   bool baselinkLaserReady = false;
   bool laserCameraReady = false;
   for( cv::FileNodeIterator it = matrixes.begin(); it != matrixes.end(); ++it )
   {
      (*it)["parent_frame"] >> parent;
      (*it)["child_frame"] >> child;
      if( parent.compare( "base_link" ) == 0 && child.compare( cameraFrameL ) == 0 )
      {
         cv::Mat baselinkCamera;
         (*it)["matrix"] >> baselinkCamera;
         copyFromMat<double>( baselinkCamera.inv(), cameraBaselink );
      }
      else if( parent.compare( cameraFrameL ) == 0 && child.compare( imuFrame ) == 0 )
      {
         cv::Mat cameraIMU;
         (*it)["matrix"] >> cameraIMU;
         copyFromMat<double>( cameraIMU.inv(), imuCamera );
      }
      else if( parent.compare( "base_link" ) == 0 && child.compare( "laser" ) == 0 )
      {
         (*it)["matrix"] >> baselinkLaser;
         baselinkLaserReady = true;
      }
      else if( parent.compare( "laser" ) == 0 && child.compare( cameraFrameL ) == 0 )
      {
         (*it)["matrix"] >> laserCamera;
         laserCameraReady = true;
      }
      else if( parent.compare( cameraFrameL ) == 0 && child.compare( cameraFrameR ) == 0 )
      {
         (*it)["matrix"] >> cameraExtrinsic;
      }
   }
   if( laserCameraReady && baselinkLaserReady )
   {
      cv::Mat baselinkCamera = baselinkLaser * laserCamera;
      copyFromMat<double>( baselinkCamera.inv(), cameraBaselink );
   }
   copyFromMat<double>( cameraExtrinsic, stereoCameraExtrinsic );
  
}

void OpenLORISReader::LoadSensor( const std::string & fileName, std::vector<SensorData> & sensorSampleSet )
{   
   std::ifstream f;
   f.open( fileName.c_str() );
   SensorData sample;
   sensorSampleSet.reserve( 10000 );
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
         ss >> tmp >> sample.x;
         ss >> tmp >> sample.y;
         ss >> tmp >> sample.z;
         sensorSampleSet.push_back( sample );
      }
   }
}
void OpenLORISReader::ReadIMUSamples( const std::string & imageListFile, const std::string & accFile, const std::string & gyroFile,
                                      std::vector<imu_pack_dsp> & imuSampleSet )
{
   std::vector<SensorData> accSet;
   std::vector<SensorData> gyroSet;

   LoadSensor( imageListFile + "/" + accFile, accSet );
   LoadSensor( imageListFile + "/" + gyroFile, gyroSet );
   imu_pack_dsp sample;
   imuSampleSet.reserve( accSet.size() );
   size_t lower = 0, upper = gyroSet.size();
   for( size_t i = 0; i < accSet.size(); i++ )
   {
      for( size_t j = lower; j < upper; j++ )
      {
         if( gyroSet[j].timestamp > accSet[i].timestamp )
         {
            lower = j;
            break;
         }
      }
      if( lower == 0 || lower == upper )
         continue;

      sample.time_acc = uint64_t( accSet[i].timestamp * 1e9 );
      sample.time_gyro = sample.time_acc;

      sample.acceloration_x = accSet[i].x;
      sample.acceloration_y = accSet[i].y;
      sample.acceloration_z = accSet[i].z;

      double ratio = accSet[i].timestamp - gyroSet[lower - 1].timestamp;
      ratio = ratio / (gyroSet[lower].timestamp - gyroSet[lower - 1].timestamp);
      sample.angular_velocity_x = (float32_t)(gyroSet[lower - 1].x * (1. - ratio) + gyroSet[lower].x * ratio);
      sample.angular_velocity_y = (float32_t)(gyroSet[lower - 1].y * (1. - ratio) + gyroSet[lower].y * ratio);
      sample.angular_velocity_z = (float32_t)(gyroSet[lower - 1].z * (1. - ratio) + gyroSet[lower].z * ratio);
      imuSampleSet.push_back( sample );
   }
}


void OpenLORISReader::ReadOdomSamples( const std::string & imageListFile, std::vector<sensor_wheel> & wheelSampleSet )
{

   std::ifstream f;
   f.open( imageListFile + "/odom.txt" );
   sensor_wheel sample;
   wheelSampleSet.reserve( 10000 );
   double stamp;
   while( !f.eof() )
   {
      std::string s;
      std::getline( f, s );
      if( !s.empty() && s[0] != '#' )
      {
         std::stringstream ss;
         ss << s;
         ss >> stamp;
         sample.timestamp = (uint64_t)(stamp*1e9);
         ss >> sample.location[0] >> sample.location[1] >> sample.location[2];
         ss >> sample.direction[0] >> sample.direction[1] >> sample.direction[2] >> sample.direction[3];
         ss >> sample.linear_velocity;
         ss >> sample.angular_velocity >> sample.angular_velocity >> sample.angular_velocity >> sample.angular_velocity >> sample.angular_velocity;
         wheelSampleSet.push_back( sample );
      }
   }
}

bool OpenLORISReader::getIMUConfiguration( rvIMUConfiguration & config )
{
   config.acceBias[0] = 0.f;
   config.acceBias[1] = 0.f;
   config.acceBias[2] = 0.f;

   config.cameraInIMU = imuCamera;

   config.deltaInSecond = 0.f;

   config.gyroBias[0] = 0.f;
   config.gyroBias[1] = 0.f;
   config.gyroBias[2] = 0.f;

   config.imuEnabled = true;

   return true;
}


bool OpenLORISReader::getWheelConfiguration( rvWheelConfiguration & config )
{
   config.wheelEnabled = true;
   config.baselinkInCamera = cameraBaselink;

   return true;
}

