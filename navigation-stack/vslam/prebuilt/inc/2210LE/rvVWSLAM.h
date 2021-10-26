/*****************************************************************************
 * @copyright
 * Copyright (c) 2021 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 * *******************************************************************************/


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
      RV_TrackedObservation * _ObservationBuf = NULL;
   } rvVWSLAMStatus;


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Initialize VWSLAM object.
   /// @param root
   ///     Pointer to VWSLAM configuration path.
   /// @param output
   ///     Pointer to VWSLAM output path.
   /// @return
   ///     Returns rvVWSLAM object pointer if succeeded, and NULL if failed
   //------------------------------------------------------------------------------
   RV_API rvVWSLAM* rvVWSLAM_Initialize(const char* root, const char* output, const rvCameraParams* cameraConfig);


   //------------------------------------------------------------------------------
  /// @detailed
  ///     Reload an existing map and initialize VWSLAM object.
  /// @param root
  ///     Pointer to VWSLAM configuration.
  /// @return
  ///     Returns rvVWSLAM object pointer if succeeded, and NULL if failed
  //------------------------------------------------------------------------------
   RV_API rvVWSLAM* rvVWSLAM_Reload(const char* root, const char* output, const rvCameraParams* cameraConfig);


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
///     Freeze VWSLAM maps.
/// @param pObj
///     Pointer to VWSLAM object.
/// @param vslamFlag
///     Whether to freeze vslam map.
/// @param vmFlag
///     Whether to freeze vm map.
//------------------------------------------------------------------------------
   RV_API void rvVWSLAM_Freeze(rvVWSLAM* pObj, const bool vslamFlag, const bool vmFlag);


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
   RV_API rvVSLAMPose rvVWSLAM_GetBaselinkPose(rvVWSLAM *pObj);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Get output pose from VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @return
   ///     VWSLAM output pose
   //------------------------------------------------------------------------------
   RV_API rvVSLAMPose rvVWSLAM_PredictBaselinkPose( rvVWSLAM *pObj, int64_t timstamp );

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
   ///     Get keyframe number of VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @return
   ///     Return number of keyframes if succeeded or 0 if failed
   //------------------------------------------------------------------------------
   RV_API int rvVWSLAM_GetKeyframeNumber( rvVWSLAM *pObj );


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Get observations of VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @param observationBuf
   ///     pointer to the buffer for observations
   /// @param bufLength
   ///     length of the buffer for observations
   /// @return
   ///     Return only number of observations if bufflengt is 0 or observationBuf is null pointer
   ///     else return the number of obsrvations and fill the buffer with observations
   //------------------------------------------------------------------------------
   RV_API int rvVWSLAM_GetVWSLAMObservations( rvVWSLAM *pObj, RV_TrackedObservation *observationBuf, int bufLength );


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
   ///     get grid image.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @param data
   ///     Address of pointer for iamge data.
   /// @param w
   ///     Address for image width.
   /// @param h
   ///     Address for image height.
   /// @param x
   ///     Address for origin X.
   /// @param y
   ///     Address for origin Y.
   /// @param r
   ///     Address for map resolution. Unit is meter
   /// @return
   ///     Timestamp of the last update for the grid map.
   //------------------------------------------------------------------------------
   RV_API long long rvVWSLAM_getGridImage(rvVWSLAM *pObj, unsigned char ** data, int* w, int* h, int* x, int* y, float* r );


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Deinitialize VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_Deinitialize(rvVWSLAM *pObj);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Get map to odom transformation
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @param pose
   ///     Pose transformation struct
   /// @return
   ///     true if success or false if failure
   //------------------------------------------------------------------------------
   RV_API bool rvVWSLAM_getMapOdomTransform( rvVWSLAM * pObj, rvPose6DYPRT & pose );

#ifdef __cplusplus
}
#endif

#endif /* ifndef RVVWSLAM_H */
