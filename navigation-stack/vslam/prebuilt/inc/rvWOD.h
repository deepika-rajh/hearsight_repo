/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef MVWOD_H
#define MVWOD_H

/***************************************************************************//**
@file
   rvWOD.h

@detailed
   Machine Vision,
   Wall Orientation Detection (WOD)

@section Overview

@section Limitations
   The following list are some of the known limitations:
   - Only tested with VSLAM.

*******************************************************************************/


//==============================================================================
// Defines
//==============================================================================


//==============================================================================
// Includes
//============================================================================== 
#include "mv.h" 
#include "rv.h"

//==============================================================================
// Declarations
//==============================================================================

#ifdef __cplusplus
extern "C"
{
#endif

   /************************************************************************//**
   @detailed
      Wall Orientation Detection (WOD)
   ****************************************************************************/
   typedef struct rvWOD rvWOD;

   /************************************************************************//**
   @detailed
      Initialize WOD.
   @param root_path
      The root path of configuration file
   @param scale
      Scale image for speed up
   @param cameraIntrisic
      Camera intrinsic parameters.
   @returns
      Pointer to WOD object; returns NULL if failed.
   ****************************************************************************/
   RV_API rvWOD* rvWOD_Initialize( const char* root_path, int scale, 
                                   const mvCameraConfiguration cameraIntrisic );

   /************************************************************************//**
   @detailed
   Deinitialize WOD object.
   @param pObj
   Pointer to WOD object.
   ****************************************************************************/
   RV_API void rvWOD_Deinitialize( rvWOD* pObj );

   /************************************************************************//**
   @detailed
      Input the image and wheel odom for post-processing
   @param pObj
      Pointer to WOD object.
   @param wheelAngle
      Yaw angle from wheel pose in wheel coordination frame.
   @param image
      Image data from camera.
   @returns
      Successful or not.
   ****************************************************************************/
   RV_API bool rvWOD_AddAngleImage( rvWOD* pObj, float& wheelAngle, 
                                    const uint8_t* image );

   /************************************************************************//**
   @detailed
      Determine the longest line orientation for the current sequence images
   @param pObj
      Pointer to WOD object.
   @param wallAngle
      Angle of the longest line.
   @returns
      Successful or not.
   ****************************************************************************/
   RV_API bool rvWOD_GetWallOrienDirection4Seq( rvWOD* pObj, float& wallAngle );

   /************************************************************************//**
   @detailed
      Show the result for the current image; Debug-only
   @param pObj
      Pointer to WOD object.
   @returns
      Successful or not.
   ****************************************************************************/
   RV_API bool rvWOD_ShowResult4EachImg( rvWOD* pObj );

   /************************************************************************//**
   @detailed
      Show the result for the sequence images; Debug-only
   @param pObj
      Pointer to WOD object.
   @returns
      Successful or not.
   ****************************************************************************/
   RV_API bool rvWOD_ShowResult4ImgSeq( rvWOD* pObj );


   /************************************************************************//**
   @detailed
      Release the memories for the current sequence;
   @param pObj
      Pointer to WOD object.
   @returns
      Successful or not.
   ****************************************************************************/
   RV_API bool rvWOD_Reset4ImgSeq( rvWOD* pObj );

#ifdef __cplusplus
}
#endif


#endif
