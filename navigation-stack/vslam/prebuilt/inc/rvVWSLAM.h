/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef RVVWSLAM_H
#define RVVWSLAM_H

/******************************************************************************
@file
   rvVWSLAM.h

@detailed
   Robot Vision,
   Visual and Wheeled SLAM (VWSLAM)

@section Overview
   This feature receives image, wheel odom and/or inertial data from sensors, 
   and generate reliable pose estimations.

@section Limitations
   The following list are some of the known limitations:
   - Only tested with mono and depth cameras 

*******************************************************************************/


//==============================================================================
// Includes
//==============================================================================

#include <rvVSLAM.h>
#include "rv.h"
#include "rvCamera.h"

//==============================================================================
// Declarations
//==============================================================================

#ifdef __cplusplus
extern "C"
{
#endif


   //==============================================================================
   /// @detailed
   ///     Visual Wheeled Simultaneous Localization And Mapping (VSLAM).
   //==============================================================================
   typedef struct VWSLAM rvVWSLAM;


   typedef struct
   {
      float _Brightness = 0.f;
      int32_t _KeyframeNum = 0;
      int32_t _MatchedMapPointNum = 0;
      int32_t _MisMatchedMapPointNum = 0;
      RV_TrackedObservation *observationBuf = NULL;
   } rvVWSLAMStatus;

   //------------------------------------------------------------------------------
   /// @detailed
   ///     Initialize VWSLAM object.
   /// @param root
   ///     Pointer to VWSLAM configuration.
   /// @return
   ///     Returns rvVWSLAM object pointer if succeeded, and NULL if failed
   //------------------------------------------------------------------------------
   RV_API rvVWSLAM *rvVWSLAM_Initialize(const char* root, const char * output, const rvCameraParams * cameraConfig, const bool doMapping);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Run VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_Run(rvVWSLAM *pObj);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Stop VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_Stop(rvVWSLAM *pObj);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Make VWSLAM object sleep
   /// @param pObj
   ///     Pointer to VWSLAM object.
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_Sleep(rvVWSLAM *pObj);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Make VWSLAM object awake
   /// @param pObj
   ///     Pointer to VWSLAM object.
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_Awake(rvVWSLAM *pObj);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Reset VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_Reset(rvVWSLAM *pObj);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Pass wheel odometry to  VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @param linearVelocity
   ///     Linear velocity
   /// @param angualVeloctiy
   ///     Angual velocity
   /// @param location
   ///     Location
   /// @param direction
   ///     Direction, euler angle
   /// @param timestamp
   ///     Timestamp in nanosecond
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_AddWheelOdom( rvVWSLAM *pObj, float linearVelocity, float angualVelocity,
                                  const float location[3], const float direction[4], int64_t timestamp );


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Pass IMU data to  VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @param linearVelocity
   ///     Linear velocity
   /// @param angualVeloctiy
   ///     Angual velocity
   /// @param timestamp
   ///     Timestamp in nanosecond
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_AddImu( rvVWSLAM *pObj, const float linearAcceleration[3], const float angularVelocity[3], int64_t timestamp );


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Pass camera frame to VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @param timeStamp
   ///     Timestamp of camera frame.
   /// @param imageBuf
   ///     Pointer to camera frame data.
   /// @param depthBuf 
   ///     Pointer to depth frame data.
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_AddImage( rvVWSLAM *pObj, const int64_t timeStamp, const uint8_t *imageBuf, const uint16_t *depthBuf );


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Get output pose from VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @return
   ///     VWSLAM output pose
   //------------------------------------------------------------------------------
   RV_API rvVSLAMPose rvVWSLAM_GetVslamOutputPose(rvVWSLAM *pObj);


   //------------------------------------------------------------------------------
/// @detailed
///     Get raw pose from VWSLAM object.
/// @param pObj
///     Pointer to VWSLAM object.
/// @return
///     VWSLAM output pose
//------------------------------------------------------------------------------
   RV_API rvVSLAMPose rvVWSLAM_GetVslamRawPose(rvVWSLAM *pObj);

   

   //------------------------------------------------------------------------------
   /// @detailed
   ///     Run VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @param img
   ///     Pointer to image buffer
   /// @param imageWidth
   ///     Image width
   /// @param imageHeight
   ///     ImageHeight
   /// @return
   ///     Return true if succeeded or false if failed
   //------------------------------------------------------------------------------
   RV_API bool rvVWSLAM_GetUndistortedImage( rvVWSLAM *pObj, uint8_t *img, int imageWidth, int imageHeight );


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Get status of VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @param status
   ///     Pass reference of status
   /// @return
   ///     Return true if succeeded or false if failed
   //------------------------------------------------------------------------------
   RV_API bool rvVWSLAM_GetVWSLAMStatus( rvVWSLAM *pObj, rvVWSLAMStatus *status );


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Pass hijack event to VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @param isHijack
   ///     Is hijack or not
   /// @param timestamp
   ///     Timestamp in nanosecond
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_AddHijack( rvVWSLAM *pObj, bool isHijack, int64_t timestamp );


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Save a map to the given path.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @param mapFolder
   ///     The folder for saving map.
   /// @param mapName
   ///     Name of Map.
   /// @return
   ///     True if the map is successfully saved. False otherwise.
   //------------------------------------------------------------------------------
   bool RV_API rvVWSLAM_SaveMap( rvVWSLAM *pObj, const char* mapFolder, const char* mapName );


   //------------------------------------------------------------------------------
   /// @detailed
   ///     get undistortorted image.
   /// @param pObj
   ///     Pointer to rvVWSLAM object, image, size of image.
   //------------------------------------------------------------------------------
   RV_API bool rvVWSLAM_getUndistortedImage(rvVWSLAM *pObj, uint8_t * img, int imageWidth, int imageHeight);

   //------------------------------------------------------------------------------
   /// @detailed
   ///     get grid image.
   /// @param pObj
   ///     Pointer to iamge data, size of image.
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_getGridImage(rvVWSLAM *pObj, unsigned char ** data, int* w, int* h);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Deinitialize VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_Deinitialize(rvVWSLAM *pObj);



#ifdef __cplusplus
}
#endif

#endif /* ifndef RVVWSLAM_H */
