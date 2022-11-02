/***************************************************************************//**
@copyright
Copyright (c) 2017-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifdef WIN32
#include "windows.h"
#define MV_INITIALIZE
#include "mv.h"
#undef MV_EXPORTS

#ifdef MV_INITIALIZE
#define MV_EXTERN /* nothing */
//HINSTANCE hLib_mv = NULL;
#else
#define MV_EXTERN extern
MV_EXTERN HINSTANCE hLib_mv;
#endif
//#include <vld.h> //for memory leak detection
#endif


#include "mvVSLAM.h"
#include "mvVM.h"
#include "mvSRW.h"

#include <stdlib.h>
#include <iostream>
#include <string>
#ifndef WIN32
#include <unistd.h>
#endif
#include "VirtualSensorDevice.h"
#include "VSLAMSystem.h"

bool debugLevel = 0;
bool RV_STDERR_LOGGING = true;

int main( int argc, char** argv )
{
   std::string sensorSetting;
   std::string algSettings;
   std::string output;
   std::string dataSet;
   int startFrame = 0;

   if( argc < 4 )
   {
      printf( "pls give path for root & output for %s\n", argv[0] );
      return -1;
   }
   else if( argc == 4 )
   {
      //Old format before June, 21, 2021
      sensorSetting = std::string( argv[1] );  //path name
      algSettings = std::string( argv[1] ) + "/Configuration/vslam.cfg";  //file name
      output = std::string( argv[2] ); //path
   }
   else
   {
       dataSet = std::string( argv[1] );
       sensorSetting = std::string( argv[2] ); //path name
       algSettings = std::string( argv[3] ); //file name
       output = std::string( argv[4] ); //path
       if (argc >= 6)
           startFrame = std::stoi(argv[5]);
   }

#ifdef WIN32
   if( mv_DLLGlue_Initialize() == false )
   {
      printf( "Failed to initialize MV pose function!\n" );
      return false;
   }
   if( mvVSLAM_DLLGlue_Initialize() == false )
   {
      printf( "Failed to initialize VSLAM DLL function!\n" );
      return false;
   }
   if( mvVM_DLLGlue_Initialize() == false )
   {
      printf( "Failed to initialize VM DLL function!\n" );
      return false;
   }
   if( mvSRW_DLLGlue_Initialize() == false )
   {
      printf( "Failed to initialize SRW DLL function!\n" );
      return false;
   }
#endif //WIN32

   char tmp = *(output.end() - 1);
   if( tmp != '/' && tmp != '\\' )
   {
      output = output + '/';
   }

   tmp = *(sensorSetting.end() - 1);
   if( tmp != '/' && tmp != '\\' )
   {
      sensorSetting = sensorSetting + '/';
   }
   std::shared_ptr<VirtualSensorDevice> inputCamera = std::make_shared<VirtualSensorDevice>( sensorSetting.c_str(), dataSet.c_str(), startFrame );
   inputCamera->addStateCallback( VSLAMSystem::state_callback );
   inputCamera->addWheelCallback( VSLAMWheel::wheelCallback );
   inputCamera->setSensorSystemSyncFunc(VSLAMSystem::isSystemWorking, VSLAMSystem::waitForRawPose);
   VSLAMSystem::imuConfiguration = inputCamera->getIMUConfiguration();
   VSLAMSystem::wheelConfiguration = inputCamera->getWheelConfiguration();
   VSLAMSystem::targetImage = inputCamera->getTargetImage();

   std::shared_ptr<VSLAMSystem> sys = VSLAMSystem::Initialize( algSettings, output, inputCamera, true );
   sys->Run();
   sys->Spin();
   sys->Quit();
   sys->deinit();
   //delete gViz;
#ifdef WIN32
   //mv_DLLGlue_Deinitialize();
#endif //WIN32

   return 0;
} 
