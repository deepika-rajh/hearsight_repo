/*******************************************************************************
@copyright
Copyright (c) 2022-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "TUMRGBDReader.h"

#include "opencv2/opencv.hpp"
#include <fstream>

TUMRGBDReader::TUMRGBDReader( const std::string & imageListFile ):
   SlamDataReader("", "", "", "")
{
   ReadImageList( imageListFile, imageList );
   cameraIdx = 0;
   if( imageListFile.find( "freiburg3" ) != std::string::npos )
   {
      cameraIdx = 3;
   }
   else if( imageListFile.find( "freiburg2" ) != std::string::npos )
   {
      cameraIdx = 2;
   }
   else if( imageListFile.find( "freiburg1" ) != std::string::npos )
   {
      cameraIdx = 1;
   }


   curImageIndex = 0;
  
}

TUMRGBDReader::~TUMRGBDReader()
{

}

bool TUMRGBDReader::SkipNextFrame()
{
    curImageIndex++;
    return curImageIndex < imageList.size();
}

bool TUMRGBDReader::GetNextFrame( mvFrame & frame, std::vector<sensor_wheel> & wheelOdomSet, std::vector<imu_pack_dsp> & imuSampleSet, std::vector<sensor_hijack> & hijackSet, std::vector<StampedSystemCallback> & callbackSet, std::vector<rvPose6DRTWithTimestamp>& poses)
{
   imuSampleSet.clear();

   //Read image from file
   if( curImageIndex >= imageList.size() )
   {
      return false;
   }
   cv::Mat image = cv::imread( imageList[curImageIndex].rgbImageName, cv::IMREAD_UNCHANGED);
   printf( "%s\n", imageList[curImageIndex].rgbImageName.c_str() );
   if( nullptr == image.data )
   {
      printf( "Cannot read image with name: %s\n", imageList[curImageIndex].rgbImageName.c_str() );
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
      AllocateMvImage( frame.leftImage, iImage.cols, image.rows, iImage.rows * iImage.step[0] );
   }
   memcpy( frame.leftImage->pixels, iImage.data, iImage.rows * iImage.cols );

   //Get the timestamp

   cv::Mat depth = cv::imread( imageList[curImageIndex].depthImageName, cv::IMREAD_UNCHANGED);
   if( frame.depthImage->height != depth.rows
       || frame.depthImage->width != depth.cols
       || frame.depthImage->pixels == NULL )
   {
      ReleaseMVImage( frame.depthImage );
      AllocateMvImage( frame.depthImage, depth.cols, depth.rows );
   }
   uint16_t* imgBuf = frame.depthImage->pixels;
   for (int i=0; i<depth.rows; i++)
      for (int j = 0; j < depth.cols; j++)
      {
         *imgBuf = depth.at<uint16_t>(i, j) / 5;
         imgBuf++;
      }
   frame.timestamp = (int64_t)(imageList[curImageIndex].timestamp*1e9);
   
   curImageIndex++;

   return true;

}

void TUMRGBDReader::ReadImageList( const std::string & dataPath, std::vector<RGBDSample> & imageNameSet )
{
   std::vector<fileWithTimestamp> rgbSamples, depthSamples;
   LoadFileSamples( dataPath + "rgb.txt", rgbSamples );
   LoadFileSamples( dataPath + "depth.txt", depthSamples );


   struct TimestampPair
   {
      size_t rgbIndex;
      size_t depthIndex;
      double diff;
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
   double invalidDif = 0.02;
   size_t rgbIndex = 0, depthIndex = 0, depthLower = 0, depthUpper = depthSamples.size();
   for( rgbIndex = 0; rgbIndex < rgbSamples.size(); rgbIndex++ )
      for( depthIndex = depthLower; depthIndex < depthUpper; depthIndex++ )
      {
         if( rgbSamples[rgbIndex].timestamp - invalidDif > depthSamples[depthIndex].timestamp )
         {
            depthLower = depthIndex;
            continue;
         }
         if( rgbSamples[rgbIndex].timestamp + invalidDif < depthSamples[depthIndex].timestamp )
         {
            break;
         }
         timePair.rgbIndex = rgbIndex;
         timePair.depthIndex = depthIndex;
         timePair.diff = fabs( rgbSamples[rgbIndex].timestamp - depthSamples[depthIndex].timestamp );
         timestampPairSet.push_back( timePair );
      }
   std::sort( timestampPairSet.begin(), timestampPairSet.end(), customLess );


   RGBDSample sample;
   imageNameSet.reserve( timestampPairSet.size() );
   for( rgbIndex = 0; rgbIndex < timestampPairSet.size(); rgbIndex++ )
   {
      if( rgbSamples[timestampPairSet[rgbIndex].rgbIndex].timestamp < 0 )
         continue;

      if( depthSamples[timestampPairSet[rgbIndex].depthIndex].timestamp < 0 )
         continue;

      sample.timestamp = rgbSamples[timestampPairSet[rgbIndex].rgbIndex].timestamp;
      sample.rgbImageName = dataPath+rgbSamples[timestampPairSet[rgbIndex].rgbIndex].filename;
      sample.depthImageName = dataPath+depthSamples[timestampPairSet[rgbIndex].depthIndex].filename;
      rgbSamples[timestampPairSet[rgbIndex].rgbIndex].timestamp = -1;
      depthSamples[timestampPairSet[rgbIndex].depthIndex].timestamp = -1;

      imageNameSet.push_back( sample );
   };
   std::sort( imageNameSet.begin(), imageNameSet.end(), customLess );
}


void TUMRGBDReader::LoadFileSamples( const std::string& filename, std::vector<fileWithTimestamp>& sampleSet )
{
   std::ifstream f;
   f.open( filename.c_str() );
   fileWithTimestamp sample;
   sampleSet.reserve( 10000 );
   while( !f.eof() )
   {
      std::string s;
      std::getline( f, s );
      if( !s.empty() && s[0] != '#' )
      {
         std::stringstream ss;
         ss << s;
         ss >> sample.timestamp;
         ss >> sample.filename;
         sampleSet.push_back( sample );
      }
   }
}


bool
TUMRGBDReader::getCameraConfiguration( rvCameraParams & config )
{
   config.imageFormat = YUV_FORMAT;
   config.cameraType = rvGrayDepth;
   config.stereo.camera[0].pixelHeight = 480;
   config.stereo.camera[0].pixelWidth = 640;
   bool result = true;
   switch( cameraIdx )
   {
      case 3:
         config.stereo.camera[0].focalLength[0] = 535.4f;
         config.stereo.camera[0].focalLength[1] = 539.2f;
         config.stereo.camera[0].principalPoint[0] = 320.1f;
         config.stereo.camera[0].principalPoint[1] = 247.6f;

         config.stereo.camera[0].distortionModel = NoDistortion;
         config.stereo.camera[0].distortion[0] = 0.f;
         config.stereo.camera[0].distortion[1] = 0.f;
         config.stereo.camera[0].distortion[2] = 0.f;
         config.stereo.camera[0].distortion[3] = 0.f;
         break;
      case 2:
         config.stereo.camera[0].focalLength[0] = 520.908620f;
         config.stereo.camera[0].focalLength[1] = 521.007327f;
         config.stereo.camera[0].principalPoint[0] = 325.141442f;
         config.stereo.camera[0].principalPoint[1] = 249.701764f;

         config.stereo.camera[0].distortionModel = Polynomial5;
         config.stereo.camera[0].distortion[0] = 0.231222f;
         config.stereo.camera[0].distortion[1] = -0.784899f;
         config.stereo.camera[0].distortion[2] = -0.003257f;
         config.stereo.camera[0].distortion[3] = -0.000105f;
         config.stereo.camera[0].distortion[4] = 0.917205f;
         break;
      case 1:
         config.stereo.camera[0].focalLength[0] = 517.306408f;
         config.stereo.camera[0].focalLength[1] = 516.469215f;
         config.stereo.camera[0].principalPoint[0] = 318.643040f;
         config.stereo.camera[0].principalPoint[1] = 255.313989f;

         config.stereo.camera[0].distortionModel = Polynomial5;
         config.stereo.camera[0].distortion[0] = 0.262383f;
         config.stereo.camera[0].distortion[1] = -0.953104f;
         config.stereo.camera[0].distortion[2] = -0.005358f;
         config.stereo.camera[0].distortion[3] = 0.002628f;
         config.stereo.camera[0].distortion[4] = 1.163314f;
         break;
      default:
         result = false;
         break;
   }
   
   return result;
}
