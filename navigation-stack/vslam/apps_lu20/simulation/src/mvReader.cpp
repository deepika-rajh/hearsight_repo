/*****************************************************************************
 * @copyright
 * Copyright (c) 2018-2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 * *******************************************************************************/

#include "MVReader.h"
#include "string.h"
#include "math.h"
#include "ParseSensorParam.h"

#include <sstream>

bool readStereoCameraParameter(const char* cameraID, rvStereoCamera& configuration, rvStereoRectCamera& outputCamera);

void copyCameraConf( const mvCameraConfiguration & mCamera, rvRectCameraConfiguration & rCamera );
void copyCameraConf_0( const mvCameraConfiguration & mCamera, rvCameraIntrinsic & rCamera );
void pose6DYPRTto6DRT( const mvPose6DYPRT* pose, rvPose6DRT* mvPose );

MVReader::MVReader( const std::string & srwPath ): SlamDataReader( (srwPath+"/wheel.txt"), (srwPath + "/hijack.txt"), (srwPath+"/SystemCallback.txt" ), (srwPath+"/cameraPose.csv"))
{
   reader = mvSRW_Reader_Initialize( srwPath.c_str() );
   if( reader == nullptr )
   {
      printf( "Error initializing the Sequence Reader Object!\n" );
      printf( "Configuration file: %s\n", srwPath.c_str() );
      configValid = false;
      return;
   }

   int cameraNum = mvSRW_Reader_GetNumberOfCameras( reader );
   cameras = new mvCameraDescriptor[cameraNum];
   if( cameras == nullptr )
   {
      mvSRW_Reader_Deinitialize( reader );
   }
   
   mvSRW_Reader_GetCameras( reader, cameras );

   imuCon.imuEnabled = false;
   wheelCon.wheelEnabled = false;

   configValid = false;

   //Disable monocular camera
   inputCamera.pixelWidth = 0;
   outputCamera.pixelWidth = 0;
   //Disable stereo camera
   stereoCamera.camera[0].pixelWidth = 0;
   stereoCamera.camera[1].pixelWidth = 0;
   //process 1st camera only
   switch (cameras->type )
   {
      case mvStereo:
         mvStereoConfiguration stereoCamera0;
         configValid = mvSRW_Reader_GetStereoParameters( reader, cameras->name, &stereoCamera0 );
         copyCameraConf_0(stereoCamera0.camera[0], stereoCamera.camera[0]);
         copyCameraConf_0(stereoCamera0.camera[1], stereoCamera.camera[1]);
         if (!configValid )
            configValid = ParseCameraParameters( srwPath, "Configuration/vslam.cfg" );
         ParseSensorParam(srwPath, "Configuration/vslam.cfg", wheelCon, imuCon, targetImage);
         break;
      case mvGrayDepth:
         configValid = mvSRW_Reader_GetCameraParameters( reader, cameras->name, &inputCamera );
         if (!configValid)
             configValid = ParseCameraParameters( srwPath, "Configuration/vslam.cfg" );
         outputCamera = inputCamera;
         ParseSensorParam(srwPath, "Configuration/vslam.cfg", wheelCon, imuCon, targetImage);
         break;
      case mvMonocular:
      default:
         configValid = mvSRW_Reader_GetCameraParameters( reader, cameras->name, &inputCamera );
         configValid = ParseCameraParameters( srwPath, "Configuration/vslam.cfg" );
         ParseSensorParam(srwPath, "Configuration/vslam.cfg", wheelCon, imuCon, targetImage);
         break;
   }

   if( !configValid )
   {
	  std::string conFile = srwPath + "cameraWheel.txt";
      ReadWheelConfiguration( conFile );
   }

}

MVReader::~MVReader()
{
   if( reader != NULL )
   {
      mvSRW_Reader_Deinitialize( reader );
   }

   if( cameras != NULL )
   {
      delete[] cameras;
   }
}


void AxisAngleToRotationMatrix( float32_t axisAngle[3], float32_t rotation[3][3] )
{
   float32_t fa = 0.5f;
   float32_t fr = 1.f;

   const float32_t squaredLen = axisAngle[0] * axisAngle[0] + axisAngle[1] * axisAngle[1] + axisAngle[2] * axisAngle[2];
   const float32_t len = sqrt( squaredLen );

   if( len > 0 )
   {
      const float32_t eps = 1e-2f;
      if( len < eps )
      {
         fa = 0.5f - squaredLen / 24;
         fr = 1 - squaredLen / 6;
      }
      else
      {
         fr = sin( len ) / len;
         fa = ((1 - cos( len )) / squaredLen);
      }
   }

   rotation[0][0] = 1.f - fa * (axisAngle[1] * axisAngle[1] + axisAngle[2] * axisAngle[2]);
   rotation[1][1] = 1.f - fa * (axisAngle[0] * axisAngle[0] + axisAngle[2] * axisAngle[2]);
   rotation[2][2] = 1.f - fa * (axisAngle[0] * axisAngle[0] + axisAngle[1] * axisAngle[1]);

   rotation[0][1] = -fr * axisAngle[2] * axisAngle[2] + fa * axisAngle[0] * axisAngle[0] * axisAngle[1] * axisAngle[1];
   rotation[0][2] =  fr * axisAngle[1] * axisAngle[1] + fa * axisAngle[0] * axisAngle[0] * axisAngle[2] * axisAngle[2];
   rotation[1][2] = -fr * axisAngle[0] * axisAngle[0] + fa * axisAngle[1] * axisAngle[1] * axisAngle[2] * axisAngle[2];
   rotation[1][0] =  fr * axisAngle[2] * axisAngle[2] + fa * axisAngle[0] * axisAngle[0] * axisAngle[1] * axisAngle[1];
   rotation[2][0] = -fr * axisAngle[1] * axisAngle[1] + fa * axisAngle[0] * axisAngle[0] * axisAngle[2] * axisAngle[2];
   rotation[2][1] =  fr * axisAngle[0] * axisAngle[0] + fa * axisAngle[1] * axisAngle[1] * axisAngle[2] * axisAngle[2];
}


bool MVReader::SkipNextFrame()
{
   return mvSRW_Reader_SkipNextFrame( reader );
}
   

bool MVReader::GetNextFrame( mvFrame & frame, std::vector<sensor_wheel> & wheelOdomSet, std::vector<imu_pack_dsp> & imuSampleSet, std::vector<sensor_hijack> & hijackSet, std::vector<StampedSystemCallback> & callbackSet, std::vector<rvPose6DRTWithTimestamp>& poses)
{
   if( reader == nullptr )
   {
      return false;
   }

   imuSampleSet.clear();

   mvFrame* currentFrame = mvSRW_Reader_GetNextFrame( reader );
   if( currentFrame == NULL )
   {
      printf( "ERROR: No frames are available\n" );
      mvSRW_Reader_Deinitialize( reader );
      return false;
   }

   frame.timestamp = currentFrame->timestamp;
   memcpy( frame.cameraName, currentFrame->cameraName, sizeof( currentFrame->cameraName ) );
    
   fillOneImage( currentFrame->leftImage, frame.leftImage );
   fillOneImage( currentFrame->rightImage, frame.rightImage );

   if( currentFrame->depthImage !=NULL && currentFrame->depthImage->pixels != NULL ) //then copy depth image if it is
   {
      if( frame.depthImage->height != currentFrame->depthImage->height
          || frame.depthImage->width != currentFrame->depthImage->width
          || frame.depthImage->pixels == NULL )
      {
         ReleaseMVImage( frame.depthImage );
         AllocateMvImage( frame.depthImage, currentFrame->depthImage->width, currentFrame->depthImage->height );
      }
      int offsetDst = frame.depthImage->memoryStride / (sizeof( uint16_t ));
      int offsetSrc = currentFrame->depthImage->memoryStride / (sizeof( uint16_t ));
      for( uint32_t i = 0; i < currentFrame->depthImage->height; ++i )
         memcpy( frame.depthImage->pixels + i*offsetDst, currentFrame->depthImage->pixels + i*offsetSrc, sizeof( uint16_t )*currentFrame->depthImage->width );      
   }
   
   mvSRW_Reader_ReleaseFrame( reader, currentFrame );

   //Get wheel odom
   GetWheelOdom( frame.timestamp + (int64_t)2e7, wheelOdomSet );
   GetPose(frame.timestamp + (int64_t)2e7, poses);
   if( mHijack->hijackFileReady() )
      GetHijack( frame.timestamp + (int64_t)2e7, hijackSet );
   if( mCallbackReader->callbackFileReady() )
      GetCallback( frame.timestamp + (int64_t)2e7, callbackSet );

   imu_pack_dsp imuSample;
   bool getIMUData;
   do
   {
      getIMUData = GetNextIMUSample( reader, frame.timestamp + (int64_t)2e7, imuSample );
      if( getIMUData ) // && imuSample.time_acc > frame.timestamp - 80e6)
      {
         imuSampleSet.push_back( imuSample );
      }

   } while( getIMUData );


   return true;

}

void MVReader::fillOneImage( const mvImage * src, mvImage * dst )
{
   if( src == NULL )
   {
      ReleaseMVImage( dst );
      return;
   }

   if( src->pixels == NULL )
   {
      ReleaseMVImage( dst );
      return;
   }

   if( dst->height != src->height
       || dst->width != src->width
       || dst->pixels == NULL )
   {
      ReleaseMVImage( dst );
      AllocateMvImage( dst, src->width, src->height, src->memoryStride );
   }
   for( uint32_t i = 0; i < src->height; ++i )
   {
      uint8_t * row = dst->pixels + i*dst->memoryStride;
      uint8_t * srcR = src->pixels + i*src->memoryStride;
      if( src->memoryStride == src->width * 3 )
      {
         //RGB to BGR
         for( uint32_t j = 0, k = 0; j < src->width; j++ )
         {
            row[k] = srcR[k + 2];
            row[k + 1] = srcR[k + 1];
            row[k + 2] = srcR[k];
            k += 3;
         }
      }
      else
      {
         memcpy( row, srcR, src->memoryStride );
      }
   }
}


bool MVReader::GetNextIMUSample( mvSRW_Reader * sequenceReader, uint64_t time, imu_pack_dsp & imuSample )
{
   auto gyroData = mvSRW_Reader_GetNextGyro( sequenceReader, time );
   if( gyroData == nullptr )
   {
      return false;
   }
   auto acceData = mvSRW_Reader_GetNextAccel( sequenceReader, time );
   if( acceData == nullptr )
   {
      mvSRW_Reader_ReleaseIMUData( sequenceReader, gyroData );
      return false;
   }

   while( gyroData->timestamp != acceData->timestamp )
   {
      if( gyroData->timestamp < acceData->timestamp )
      {
         mvSRW_Reader_ReleaseIMUData( sequenceReader, gyroData );
         gyroData = mvSRW_Reader_GetNextGyro( sequenceReader, time );
         if( nullptr == gyroData )
         {
            mvSRW_Reader_ReleaseIMUData( sequenceReader, acceData );
            return false;
         }
      }
      else
      {
         mvSRW_Reader_ReleaseIMUData( sequenceReader, acceData );
         acceData = mvSRW_Reader_GetNextAccel( sequenceReader, time );
         if( acceData == nullptr )
         {
            mvSRW_Reader_ReleaseIMUData( sequenceReader, gyroData );
            return false;
         }
      }

   }

   imuSample.time_acc = acceData->timestamp;
   imuSample.time_gyro = gyroData->timestamp;
   imuSample.angular_velocity_x = (float) gyroData->x;
   imuSample.angular_velocity_y = (float)gyroData->y;
   imuSample.angular_velocity_z = (float)gyroData->z;
   imuSample.acceloration_x = (float)acceData->x;
   imuSample.acceloration_y = (float)acceData->y;
   imuSample.acceloration_z = (float)acceData->z;

   mvSRW_Reader_ReleaseIMUData( sequenceReader, acceData );
   mvSRW_Reader_ReleaseIMUData( sequenceReader, gyroData );
   return true;
}

bool MVReader::getCameraConfiguration( rvCameraParams & config )
{
   if( !configValid )
      return false;

   config.imageFormat = Y_ONLY_FORMAT;
   switch (cameras->type)
   case mvStereo:
   {
      config.cameraType = rvStereo;
      config.stereo = stereoCamera;
      config.stereoRect = rectStereoCamera;
      break;
   case mvMonocular:
      config.cameraType = rvMonocular;
      copyCameraConf_0( inputCamera, config.stereo.camera[0] );
      copyCameraConf( outputCamera, config.stereoRect.camera[0] );
      break;
   case mvGrayDepth:
   default:
      config.cameraType = rvGrayDepth;
      copyCameraConf_0( inputCamera, config.stereo.camera[0] );
      config.stereoRect.camera->pixelHeight = config.stereo.camera[0].pixelHeight;
      config.stereoRect.camera->pixelWidth = config.stereo.camera[0].pixelWidth;
      config.stereoRect.camera->initialized = false;
      break;
   }

   return true;
}

void copyCameraConf( const mvCameraConfiguration & mCamera, rvRectCameraConfiguration & rCamera )
{
   rCamera.initialized = true;
   rCamera.pixelWidth = mCamera.pixelWidth;
   rCamera.pixelHeight = mCamera.pixelHeight;
   for( size_t i = 0; i < 3; i++ )
   {
      for( size_t j = 0; j < 3; j++ )
      {
         rCamera.P[i][j] = 0;
         rCamera.R[i][j] = 0;
      }
      rCamera.P[i][3] = 0;
   }
   rCamera.P[0][2] = (double)mCamera.principalPoint[0];
   rCamera.P[1][2] = (double)mCamera.principalPoint[1];
   rCamera.P[0][0] = (double)mCamera.focalLength[0];
   rCamera.P[1][1] = (double)mCamera.focalLength[1];
   rCamera.P[2][2] = (double)1;

   rCamera.R[0][0] = 1.;
   rCamera.R[1][1] = 1.;
   rCamera.R[2][2] = 1.;
}


bool MVReader::getWheelConfiguration( rvWheelConfiguration & config )
{
    config = wheelCon;
    return true;
}

void MVReader::ReadWheelConfiguration( std::string & fileName )
{
   wheelCon.wheelEnabled = false;
   mvPose6DYPRT poseVB;
   std::ifstream cfg( fileName, std::ifstream::in );
   if( !cfg.is_open() )
   {
      printf( "Fail to open configuration file: %s\n", fileName.c_str() );
      return;
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

      if( itemName == "translation" )
      {
         iss >> poseVB.translation[0] >> poseVB.translation[1] >> poseVB.translation[2];
      }
      else if( itemName == "rotation" )
      {
         iss >> poseVB.euler[0] >> poseVB.euler[1] >> poseVB.euler[2];
      }
   }

   pose6DYPRTto6DRT( &poseVB, &wheelCon.baselinkInCamera );
   wheelCon.wheelEnabled = true;
   return;
   
}

void pose6DYPRTto6DRT( const mvPose6DYPRT* pose, rvPose6DRT* mvPose )
{
   const float * euler = pose->euler;
   float32_t cr = (float32_t)cos( euler[0] );
   float32_t sr = (float32_t)sin( euler[0] );
   float32_t cp = (float32_t)cos( euler[1] );
   float32_t sp = (float32_t)sin( euler[1] );
   float32_t cy = (float32_t)cos( euler[2] );
   float32_t sy = (float32_t)sin( euler[2] );
   mvPose->matrix[0][0] = cy*cp;
   mvPose->matrix[0][1] = cy*sp*sr - sy*cr;
   mvPose->matrix[0][2] = cy*sp*cr + sy*sr;
   mvPose->matrix[1][0] = sy*cp;
   mvPose->matrix[1][1] = sy*sp*sr + cy*cr;
   mvPose->matrix[1][2] = sy*sp*cr - cy*sr;
   mvPose->matrix[2][0] = -sp;
   mvPose->matrix[2][1] = cp*sr;
   mvPose->matrix[2][2] = cp*cr;

   mvPose->matrix[0][3] = pose->translation[0];
   mvPose->matrix[1][3] = pose->translation[1];
   mvPose->matrix[2][3] = pose->translation[2];
}

bool MVReader::ParseCameraParameters( const std::string & root, const std::string & configFile )
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
      if( itemName.compare( "Camera" ) == 0 )
      {
         std::string cameraID;
         iss >> cameraID;
         GetCameraParameter( (tempRoot+cameraID).c_str(), inputCamera, outputCamera );
         printf( "Using camera ID:       %s\n", cameraID.c_str() );
      }
      else if( itemName.compare( "Stereo" ) == 0 )
      {
         std::string cameraID;
         iss >> cameraID;
         cameras->type = mvStereo;
         readStereoCameraParameter( (tempRoot + cameraID).c_str(), stereoCamera, rectStereoCamera );
         printf( "Using camera ID:       %s\n", cameraID.c_str() );
      }
   }
   
   return true;
}

bool MVReader::GetCameraParameter( const char *cameraID, mvCameraConfiguration & configuration, mvCameraConfiguration & outputCamera )
{
   std::string fullName = cameraID;
   std::ifstream cfg( fullName, std::ifstream::in );
   if( !cfg.is_open() )
   {
      printf( "Fail to open configuration file: %s\n", fullName.c_str() );
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
      if( itemName.compare( "image_width:" ) == 0 )
      {
         iss >> configuration.pixelWidth;
      }
      else if( itemName.compare( "image_height:" ) == 0 )
      {
         iss >> configuration.pixelHeight;
      }
      else if( itemName.compare( "camera_matrix:" ) == 0 )
      {
         float matrix[9];
         ReadMatrix( cfg, matrix );
         configuration.focalLength[0] = matrix[0];
         configuration.focalLength[1] = matrix[4];
         configuration.principalPoint[0] = matrix[2];
         configuration.principalPoint[1] = matrix[5];
      }
      else if( itemName.compare( "distortion_coefficients:" ) == 0 )
      {
         memset( configuration.distortion, 0, sizeof( configuration.distortion ) );
         float distortion[8];
         ReadMatrix( cfg, distortion );
         for( size_t i = 0; i < 8; i++ )
         {
            configuration.distortion[i] = (float64_t)distortion[i];
         }
      }
      else if( itemName.compare( "distortion_model:" ) == 0 )
      {
         std::string distortionModelName;
         iss >> distortionModelName;
         if( distortionModelName.compare( "fisheye" ) == 0 )
         {
            configuration.distortionModel = 10;
         }
         else
         {
            configuration.distortionModel = 8;
         }
      }

      else if( itemName.compare( "projection_matrix:" ) == 0 )
      {
         float p[12];
         ReadMatrix( cfg, p );
         outputCamera.focalLength[0] = p[0];
         outputCamera.focalLength[1] = p[5];
         outputCamera.principalPoint[0] = p[2];
         outputCamera.principalPoint[1] = p[6];
      }
      else if( itemName.compare( "project_image_width:" ) == 0 )
      {
         iss >> outputCamera.pixelWidth;
      }
      else if( itemName.compare( "project_image_height:" ) == 0 )
      {
         iss >> outputCamera.pixelHeight;
      }
   }
   configuration.memoryStride = configuration.pixelWidth;
   configuration.uvOffset = 0;
   memset( outputCamera.distortion, 0, sizeof( outputCamera.distortion ) );
   outputCamera.distortionModel = 0;
   outputCamera.memoryStride = outputCamera.pixelWidth;
   outputCamera.uvOffset = 0;
   return true;
}




void copyCameraConf_0( const mvCameraConfiguration & mCamera, rvCameraIntrinsic & rCamera )
{
   rCamera.pixelWidth = mCamera.pixelWidth;
   rCamera.pixelHeight = mCamera.pixelHeight;
   rCamera.pixelStride = mCamera.memoryStride;
   rCamera.principalPoint[0] = (float32_t)mCamera.principalPoint[0];
   rCamera.principalPoint[1] = (float32_t)mCamera.principalPoint[1];
   rCamera.focalLength[0] = (float32_t)mCamera.focalLength[0];
   rCamera.focalLength[1] = (float32_t)mCamera.focalLength[1];
   for (size_t i=0; i<8; i++ )
      rCamera.distortion[i] = (float32_t)mCamera.distortion[i];

   size_t distortionLength = 0;
   switch( mCamera.distortionModel )
   {
      case 0:
         rCamera.distortionModel = rvDistortionModel::NoDistortion;
         distortionLength = 0;
         break;
      case 4:
         rCamera.distortionModel = rvDistortionModel::Polynomial4;
         distortionLength = 4;
         break;
      case 5:
         rCamera.distortionModel = rvDistortionModel::Polynomial5;
         distortionLength = 5;
         break;
      case 8:
         rCamera.distortionModel = rvDistortionModel::RationalModel8;
         distortionLength = 8;
         break;
      case 10:
         rCamera.distortionModel = rvDistortionModel::FisheyeModel4;
         distortionLength = 4;
      default:
         break;
   }

   for( size_t i = 0; i < distortionLength; i++ )
   {
      rCamera.distortion[i] = (float32_t)mCamera.distortion[i];
   }
}

