/***************************************************************************//**
@copyright
Copyright (c) 2017-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

//#include "mvVSLAM.h"
#include "mvVM.h"
#include "mvSRW.h"

#include <stdlib.h>
#include <iostream>
#include <string>
#ifndef WIN32
#include <unistd.h>
#endif
#include "VirtualSensorDevice.h"
#include "VMSystem.h"

bool debugLevel = 0;
int RV_LOG_LEVEL = 1;
bool RV_STDERR_LOGGING = true;

int main(int argc, char** argv)
{
    std::string sensorSetting;
    std::string algSettings;
    std::string dataSet;
    int startFrame = 0;

    if (argc < 4)
    {
        printf("pls give path for data type, data path and config file path for %s\n", argv[0]);
        return -1;
    }
    else if (argc == 4)
    {
        dataSet = std::string(argv[1]);     //data type
        sensorSetting = std::string(argv[2]); //data path
        algSettings = std::string(argv[3]); //config file path
    }

#ifdef WIN32
    if (mv_DLLGlue_Initialize() == false)
    {
        printf("Failed to initialize MV pose function!\n");
        return false;
    }
    if (mvVM_DLLGlue_Initialize() == false)
    {
        printf("Failed to initialize VM DLL function!\n");
        return false;
    }
    if (mvSRW_DLLGlue_Initialize() == false)
    {
        printf("Failed to initialize SRW DLL function!\n");
        return false;
    }
#endif //WIN32

    char tmp = *(sensorSetting.end() - 1);
    if (tmp != '/' && tmp != '\\')
    {
        sensorSetting = sensorSetting + '/';
    }
    std::shared_ptr<VirtualSensorDevice> inputCamera = std::make_shared<VirtualSensorDevice>(sensorSetting.c_str(), dataSet.c_str(), startFrame);
    inputCamera->addStateCallback(VMSystem::state_callback);
    inputCamera->addPoseCallback(VMSystem::addCameraPose);

    std::shared_ptr<VMSystem> sys = VMSystem::Initialize(algSettings, inputCamera, true);
    sys->Run();
    sys->Spin();
    sys->Quit();
    sys->deinit();

#ifdef WIN32
   //mv_DLLGlue_Deinitialize();
#endif //WIN32

    return 0;
}
