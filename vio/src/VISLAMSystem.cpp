/*****************************************************************************
Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries. 
All rights reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>

#include <thread>
#include <signal.h>
#include <functional>
#include <string.h>
#include <math.h>

#include "VISLAMSystem.h"
#include "SystemTime.h"

#include <sstream>
#include <fstream>


#include <rvQueue.h>


//static members definition
rvVIOHandle * VISLAMSystem::vioPtr = nullptr;
rvVISLAMMapPoint* VISLAMSystem::pPoints = nullptr;
int VISLAMSystem::rvVIOPointsNum = 0;
std::shared_ptr<VISLAMSystem> VISLAMSystem::t = nullptr;
OutputRecorder VISLAMSystem::recorder;
VISLAMSystem::SystemState VISLAMSystem::systemState = KSLEEPING;
std::shared_ptr<Visualiser> VISLAMSystem::viz = nullptr;
std::string VISLAMSystem::sensorPath = "";
std::string VISLAMSystem::algConfFile = "";
std::string VISLAMSystem::outputPath = "";
rvCameraParams VISLAMSystem::cameraConfiguration;
rvIMUConfiguration VISLAMSystem::imuConfiguration;
rvWheelConfiguration VISLAMSystem::wheelConfiguration;

std::shared_ptr<CameraInterface> VISLAMSystem::inputCamera = nullptr;

/************************************** C APIs start ************/
void Euler2Quaternion( double roll, double pitch, double yaw, double quaternion[4] )
{
   double t0 = cos( yaw * 0.5 );
   double t1 = sin( yaw * 0.5 );
   double t2 = cos( roll * 0.5 );
   double t3 = sin( roll * 0.5 );
   double t4 = cos( pitch * 0.5 );
   double t5 = sin( pitch * 0.5 );

   quaternion[0] = t0 * t2 * t4 + t1 * t3 * t5;  //w
   quaternion[1] = t0 * t3 * t4 - t1 * t2 * t5;  //x
   quaternion[2] = t0 * t2 * t5 + t1 * t3 * t4;  //y
   quaternion[3] = t1 * t2 * t4 - t0 * t3 * t5;  //z
}


void Matrix2Quaternion( const float32_t a[3][4], float& qw, float& qx, float& qy, float& qz )
{  
   // I removed + 1.0f; see discussion with Ethan
   float trace = a[0][0] + a[1][1] + a[2][2];
   if( trace > 0 )
   {
      // I changed M_EPSILON to 0
      float s = 0.5f / sqrtf( trace + 1.0f );
      qw = 0.25f / s;
      qx = (a[2][1] - a[1][2]) * s;
      qy = (a[0][2] - a[2][0]) * s;
      qz = (a[1][0] - a[0][1]) * s;
   }
   else
   {
      if( a[0][0] > a[1][1] && a[0][0] > a[2][2] )
      {
         float s = 2.0f * sqrtf( 1.0f + a[0][0] - a[1][1] - a[2][2] );
         qw = (a[2][1] - a[1][2]) / s;
         qx = 0.25f * s;
         qy = (a[0][1] + a[1][0]) / s;
         qz = (a[0][2] + a[2][0]) / s;
      }
      else if( a[1][1] > a[2][2] )
      {
         float s = 2.0f * sqrtf( 1.0f + a[1][1] - a[0][0] - a[2][2] );
         qw = (a[0][2] - a[2][0]) / s;
         qx = (a[0][1] + a[1][0]) / s;
         qy = 0.25f * s;
         qz = (a[1][2] + a[2][1]) / s;
      }
      else
      {
         float s = 2.0f * sqrtf( 1.0f + a[2][2] - a[0][0] - a[1][1] );
         qw = (a[1][0] - a[0][1]) / s;
         qx = (a[0][2] + a[2][0]) / s;
         qy = (a[1][2] + a[2][1]) / s;
         qz = 0.25f * s;
      }
   }
}

OutputRecorder::OutputRecorder()
{
   vioFp = NULL;
   vioFpTxt = NULL;
   fullStateFp = NULL;
}

void OutputRecorder::initialize(const char* path)
{
   deinit();

   std::string outputDir(path);
   vioFp = fopen((outputDir + "vio_output" + ".csv").c_str(), "wt");
   vioFpTxt = fopen((outputDir + "stamped_traj_estimate" + ".txt").c_str(), "wt");
   if (vioFpTxt)
      fprintf(vioFpTxt, "# timestamp tx ty tz qx qy qz qw\n");
   fullStateFp = fopen((outputDir + "fullState.csv").c_str(), "wt");
   if (fullStateFp)
      fprintf(fullStateFp, "%% timestampCam timestampIMU aBias[0] aBias[1] aBias[2] wBias[0] wBias[1] wBias[2]\n");
}

void OutputRecorder::deinit()
{
   if (vioFp)
   {
      fclose(vioFp);
      vioFp = NULL;
   }
   if (vioFpTxt)
   {
      fclose(vioFpTxt);
      vioFpTxt = NULL;
   }
   if (fullStateFp)
   {
      fclose(fullStateFp);
      fullStateFp = NULL;
   }
}

OutputRecorder::~OutputRecorder()
{
   deinit();
}

void OutputRecorder::write( int64_t timestamp, const rvVISLAMPose & pose)
{
   float qw, qx, qy, qz;
   Matrix2Quaternion(pose.bodyPose.matrix, qw, qx, qy, qz);
   if (vioFp)
      fprintf(vioFp, "%" PRId64 " %f %f %f %f %f %f %f %d %d\n", pose.time, pose.bodyPose.matrix[0][3], pose.bodyPose.matrix[1][3], pose.bodyPose.matrix[2][3],
         qx, qy, qz, qw, pose.poseQuality, pose.errorCode);
   if (vioFpTxt)
      fprintf(vioFpTxt, "%" PRId64 " %f %f %f %f %f %f %f\n", pose.time, pose.bodyPose.matrix[0][3], pose.bodyPose.matrix[1][3], pose.bodyPose.matrix[2][3],
         qx, qy, qz, qw);
   if (fullStateFp)
      fprintf(fullStateFp, "%" PRId64 " %" PRId64 " %f %f %f %f %f %f\n", timestamp, pose.time, pose.aBias[0], pose.aBias[1], pose.aBias[2],
         pose.wBias[0], pose.wBias[1], pose.wBias[2]); 
}


/**********************   C APIs end   ************************************/

VISLAMSystem::~VISLAMSystem()
{
}

void VISLAMSystem::deinit()
{
    t->deinit0();
}

void VISLAMSystem::deinit0()
{
   if(vioPtr)
   {
      rvVIO_Deinitialize(vioPtr);
      delete [] pPoints;
   }

   inputCamera = nullptr;
}

VISLAMSystem::VISLAMSystem( std::shared_ptr<CameraInterface> & camera )
{
   systemState = KSLEEPING;

   inputCamera = camera;
   if( inputCamera )
   {
      inputCamera->addCallback(addImageToVslam);
      inputCamera->start();
      cameraConfiguration = inputCamera->getCameraConfiguration();
   }
}

void EulerToSO3_1( const float32_t* euler, float32_t* rotation )
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

bool VISLAMSystem::loadWheelConfiguration( const char * configFile )
{
   bool crossT = false, crossR = false;
   std::ifstream cfg( configFile, std::ifstream::in );

   if( !cfg.is_open() )
   {
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
      itemName.clear();
      iss >> itemName;
      if( itemName.compare( "WEF.Tvb" ) == 0 )
      {
         float translation[3];
         iss >> translation[0] >> translation[1] >> translation[2];
         wheelConfiguration.baselinkInCamera.matrix[0][3] = translation[0];
         wheelConfiguration.baselinkInCamera.matrix[1][3] = translation[1];
         wheelConfiguration.baselinkInCamera.matrix[2][3] = translation[2];
         crossT = true;
      }
      else if( itemName.compare( "WEF.Rvb" ) == 0 )
      {
         float euler[3];
         iss >> euler[0] >> euler[1] >> euler[2];

         float rotation[9];
         EulerToSO3_1( euler, rotation );
         memcpy( wheelConfiguration.baselinkInCamera.matrix[0], rotation + 0, sizeof( float ) * 3 );
         memcpy( wheelConfiguration.baselinkInCamera.matrix[1], rotation + 3, sizeof( float ) * 3 );
         memcpy( wheelConfiguration.baselinkInCamera.matrix[2], rotation + 6, sizeof( float ) * 3 );
         crossR = true;
      }
   }
   if( crossR && crossT )
   {
      wheelConfiguration.wheelEnabled = true;
   }
   return true;
}

std::shared_ptr<VISLAMSystem> VISLAMSystem::Initialize(const std::string& algSetting, const std::string& outputDir)
{
    algConfFile = algSetting;
    outputPath = outputDir;

    rvVIOCfg vioCfg;
    vioCfg.rvCameraCfg = &cameraConfiguration.stereo.camera[0];

    vioCfg.tbc[0] = imuConfiguration.cameraInIMU.matrix[0][3];
    vioCfg.tbc[1] = imuConfiguration.cameraInIMU.matrix[1][3];
    vioCfg.tbc[2] = imuConfiguration.cameraInIMU.matrix[2][3];

    cv::Mat rMat( 3, 3, CV_32FC1 );
    for( size_t i = 0; i < 3; i++ )
    {
        for( size_t j = 0; j < 3; j++ )
        {
            rMat.at<float32_t>( i, j ) = imuConfiguration.cameraInIMU.matrix[i][j];
        }
    }

    cv::Mat rMat0( 3, 1, CV_32FC1 );
    cv::Rodrigues( rMat, rMat0 );
    vioCfg.ombc[0] = rMat0.at<float32_t>( 0 );
    vioCfg.ombc[1] = rMat0.at<float32_t>( 1 );
    vioCfg.ombc[2] = rMat0.at<float32_t>( 2 );


    vioCfg.delta = imuConfiguration.deltaInSecond; //-0.0068f

    vioCfg.std0Delta = 0.001f;   // firmware/driver upgrades may affect the time alignment

    vioCfg.std0Tbc[0] = 0.005f;
    vioCfg.std0Tbc[1] = 0.005f;
    vioCfg.std0Tbc[2] = 0.005f;

    vioCfg.std0Ombc[0] = 0.04f; //0.05f
    vioCfg.std0Ombc[1] = 0.04f; //0.05f
    vioCfg.std0Ombc[2] = 0.04f; //0.05f

    vioCfg.accelMeasRange = 156.f;
    vioCfg.gyroMeasRange = 34.f;

    vioCfg.stdAccelMeasNoise = 0.316227766016838f; // sqrt(1e-1);
    vioCfg.stdGyroMeasNoise = 1e-2f; // sqrt(1e-4);

    vioCfg.stdCamNoise = 100.f;
    vioCfg.minStdPixelNoise = 0.5f;
    vioCfg.failHighPixelNoiseScaleFactor = 1.6651f;

    vioCfg.logDepthBootstrap = 0.f;
    vioCfg.useLogCameraHeight = false;
    vioCfg.logCameraHeightBootstrap = -3.22f;

    vioCfg.noInitWhenMoving = false; // true;
    vioCfg.limitedIMUbWtrigger = 35.f;

    vioCfg.algConfigPath = algSetting;
    vioPtr = rvVIO_Initialize( &vioCfg );
    rvVIOPointsNum = 200;//100
    pPoints = new rvVISLAMMapPoint[rvVIOPointsNum];

    recorder.initialize(outputDir.c_str());

    switch(cameraConfiguration.cameraType)
    {
        case rvStereo:      
            viz = std::make_shared<Visualiser>( cameraConfiguration.stereo.camera[0].pixelWidth, cameraConfiguration.stereo.camera[0].pixelHeight );
            break;
        case rvGrayDepth:
            viz = std::make_shared<Visualiser>( cameraConfiguration.stereo.camera[0].pixelWidth, cameraConfiguration.stereo.camera[0].pixelHeight );
            break;
        case rvMonocular:
        default:
            viz = std::make_shared<Visualiser>( cameraConfiguration.stereoRect.camera[0].pixelWidth, cameraConfiguration.stereoRect.camera[0].pixelHeight );
    }

   return t;
}

void VISLAMSystem::Run()
{
   if( systemState == KSTOPPING )
   {
      return;
   }

   printf("VIO OK\n");
   systemState = KWORKING;
}

void VISLAMSystem::sleep(bool isCloseCamera)
{
   if(systemState == KSLEEPING)
      return;

   systemState = KSLEEPING;
}

void VISLAMSystem::awake(bool isStartCamera)
{
   if(systemState == KSLEEPING)
   {
       systemState = KWORKING;
   }
}

void VISLAMSystem::reset()
{
   if( systemState == KSLEEPING )
   {
      systemState = KWORKING;
   }
}

static uint64_t lastPoseTimeStamp = 0;
static int isInitDone = 0;

void VISLAMSystem::addImageToVslam( const int64_t timestamp, const uint8_t * imageBuf, const uint16_t * depthBuf )
{
   if( VISLAMSystem::systemState == KSLEEPING)
   {
      return;
   }

   printf("got an image\n");
   if( !isInitDone )
   {
      system("echo vio Initialization is finished > /dev/kmsg");
      isInitDone = true;
   }

   if( vioPtr )
   {
      rvVIO_AddImage( vioPtr, timestamp, imageBuf );
      rvVISLAMPose pose = rvVIO_GetPose( vioPtr );
      recorder.write(timestamp, pose);
      int pointNum = rvVIO_HasUpdatedPointCloud( vioPtr );
      rvVIO_GetPointCloud( vioPtr, pPoints, rvVIOPointsNum );

      viz->ShowVIOPoints(imageBuf, pose.poseQuality, "VIO", pointNum, pPoints);

      if (pose.poseQuality >= RV_VSLAM_TRACKING_STATE_GREAT)
      {
          t->pub_camera_raw_pose(pose);
      }

   }
}

void
VISLAMSystem::addIMU( const float linearAcceleration[3], const float angularVelocity[3], int64_t timestamp )
{
   if( vioPtr )
   {
      rvVIO_AddAccel( vioPtr, timestamp, linearAcceleration[0], linearAcceleration[1], linearAcceleration[2] );
      rvVIO_AddGyro( vioPtr, timestamp, angularVelocity[0], angularVelocity[1], angularVelocity[2] );
   }
}


void VISLAMSystem::Spin()
{
   while( systemState != KSTOPPING )
   {
       VSLAM_SLEEP( 10 );
   }
}

void VISLAMSystem::state_callback(const std::string & msg)
{
    printf("vio received state change msg: %s\n", msg.c_str());

    if (!strcmp(msg.c_str(), "stop"))
    {
        printf("vio received stop sig\n");
        Stop(SIGINT);
    }
    else if (!strcmp(msg.c_str(), "sleep")) {
        printf("vio received sleep sig\n");
        sleep(false);
    }
    else if (!strcmp(msg.c_str(), "awake")) {
        printf("vio received awake sig\n");
        awake();
    }
    else if (!strcmp(msg.c_str(), "reset")) {
        printf("vio received reset sig\n");
        reset();
    }
}

void VISLAMSystem::Quit( void )
{
    //yk
    if(inputCamera)
    {
        inputCamera->stop();
    }
}

void VISLAMSystem::waitForRawPose()
{
}
