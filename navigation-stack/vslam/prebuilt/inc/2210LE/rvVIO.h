/*****************************************************************************
 * @copyright
 * Copyright (c) 2021 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 * *******************************************************************************/


#ifndef RVVIO_H
#define RVVIO_H

/***************************************************************************//**
@file
   rvVIO.h

@detailed
   Robot Vision,

*******************************************************************************/


//==============================================================================
// Includes
//==============================================================================

#include <rv.h>

//==============================================================================
// Declarations
//==============================================================================

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float32_t              tbc[3];
        float32_t              ombc[3];
        float32_t              delta;
        float32_t              std0Delta;
        float32_t              std0Tbc[3];
        float32_t              std0Ombc[3];
        float32_t              accelMeasRange;
        float32_t              gyroMeasRange;
        float32_t              stdAccelMeasNoise;
        float32_t              stdGyroMeasNoise;
        float32_t              stdCamNoise;
        float32_t              minStdPixelNoise;
        float32_t              failHighPixelNoiseScaleFactor;
        float32_t              logDepthBootstrap;
        float32_t              logCameraHeightBootstrap;
        float32_t              limitedIMUbWtrigger;
        bool                   noInitWhenMoving;
        bool                   useLogCameraHeight;
        rvCameraConfiguration  *rvCameraCfg;  
    }rvVIOCfg;

   //==============================================================================
   /// @detailed
   ///     VIO Handle.
   //==============================================================================
   typedef struct
   {
       void* rvHandle;
   }rvVIOHandle;

 /************************************************************************//**
   @param VIOCfg
      Pointer to configuration
   @return
      Pointer to VIO object; returns NULL if failed
   ****************************************************************************/
   RV_API rvVIOHandle* rvVIO_Initialize(rvVIOCfg *vioCfg);

 /************************************************************************//**
   @brief
      Run rv VIO
   ****************************************************************************/
   bool RV_API rvVIO_Execute(rvVIOHandle* pHandle, const std::string& sequenceDataDirectory);

   /// @detailed
   ///     Deinitialize VIO object.
   //------------------------------------------------------------------------------
   void RV_API rvVIO_Deinitialize(rvVIOHandle* pHandle);
   

#ifdef __cplusplus
}
#endif


#endif
