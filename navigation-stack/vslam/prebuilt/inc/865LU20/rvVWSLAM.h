/*****************************************************************************
@copyright
Copyright (c) 2019-2023 Qualcomm Technologies, Inc.
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


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Initialize VWSLAM object.
   /// @param algConfile
   ///     Absolute path as text string to algorithm configuration file.
   /// @param output
   ///     Absolute path as text string to VWSLAM output folder, mainly for output pose logging
   /// @param cameraConfig
   ///     Pointer to the camera configuration
   /// @param wheelConfig
   ///     Pointer to the wheel configuration
   /// @param imuConfig
   ///     Pointer to the IMU configuration
   /// @param targetImage
   ///     Pointer to the target image configuration
   /// @return
   ///     Returns rvVWSLAM object pointer if succeeded, and NULL if failed
   //------------------------------------------------------------------------------
   RV_API rvVWSLAM *rvVWSLAM_Initialize( const char * algConfile, const char * output,
                                         const rvCameraParams * cameraConfig, const rvWheelConfiguration * wheelConfig, 
                                         const rvIMUConfiguration * imuConfig, const rvTargetImage * targetImage);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Reload an existing map and initialize VWSLAM object.
   /// @param root
   ///     Absolute path as text string to algorithm configuration file.
   /// @param output
   ///     Absolute path as text string to VWSLAM output folder, mainly for output pose logging
   /// @param cameraConfig
   ///     Pointer to camera configuration
   /// @param wheelConfig
   ///     Pointer to the wheel configuration
   /// @param imuConfig
   ///     Pointer to the IMU configuration
   /// @param targetImage
   ///     Pointer to the target image configuration
   /// @return
   ///     Returns rvVWSLAM object pointer if succeeded, and NULL if failed
   //------------------------------------------------------------------------------
   RV_API rvVWSLAM* rvVWSLAM_Reload( const char * algConfile, const char* output,
                                     const rvCameraParams * cameraConfig, const rvWheelConfiguration * wheelConfig,
                                     const rvIMUConfiguration* imuConfig, const rvTargetImage* targetImage);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Start VWSLAM engine to work normally
   /// @param pObj
   ///     Pointer to VWSLAM object.
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_Run(rvVWSLAM *pObj);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Stop VWSLAM engine.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_Stop(rvVWSLAM *pObj);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Make the running VWSLAM engine sleepping, i.e. return directly without processing inputs
   /// @param pObj
   ///     Pointer to VWSLAM object.
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_Sleep(rvVWSLAM *pObj);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Awake the sleeping VWSLAM engine to process coming inputs 
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
   ///     Enable or disable the map updating of VWSLAM.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @param vslamFlag
   ///     Whether to freeze vslam map.
   /// @param vmFlag
   ///     Whether to freeze the grid map.
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_Freeze(rvVWSLAM* pObj, const bool vslamFlag);


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
   ///     Direction in quaternion [x, y, z, w]
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
   ///     Pointer to camera gray images. For monocular or depth cameras, the buffer is filled with one gray image row by row
   ///     For stereo camera, the buffer is filled with left image row by row first and then right image row by row.
   /// @param depthBuf 
   ///     Pointer to depth frame data.
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_AddImage( rvVWSLAM *pObj, const int64_t timeStamp, const uint8_t *imageBuf, const uint16_t *depthBuf );


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Get the latest baselink's pose estimated by VWSLAM object
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @return
   ///     VWSLAM output pose
   //------------------------------------------------------------------------------
   RV_API rvVSLAMPose rvVWSLAM_GetBaselinkPose(rvVWSLAM *pObj);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Get baselink's pose predicted by VWSLAM object of a given time, which is usually after the 
   ///     last image frame passed to the object
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @param timestamp
   ///     Timestamp of the pose
   /// @return
   ///     VWSLAM output pose
   //------------------------------------------------------------------------------
   RV_API rvVSLAMPose rvVWSLAM_PredictBaselinkPose( rvVWSLAM *pObj, int64_t timstamp );


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Get camera pose of the last image frame's timestamp from VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @return
   ///     VWSLAM output pose
   //------------------------------------------------------------------------------
   RV_API rvVSLAMPose rvVWSLAM_GetVslamRawPose(rvVWSLAM *pObj);

   

   //------------------------------------------------------------------------------
   /// @detailed
   ///     Get undistorted image of last input image from VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @param img
   ///     Pointer to image buffer
   /// @param imageWidth
   ///     Image width
   /// @param imageHeight
   ///     Image height
   /// @return
   ///     Return true if succeeded or false otherwise
   //------------------------------------------------------------------------------
   RV_API bool rvVWSLAM_GetUndistortedImage( rvVWSLAM *pObj, uint8_t *img, int imageWidth, int imageHeight );


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Get the number of keyframes in the map.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @return
   ///     Return number of keyframes if succeeded or 0 otherwise
   //------------------------------------------------------------------------------
   RV_API int rvVWSLAM_GetKeyframeNumber( rvVWSLAM *pObj );


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Get observations in the last image from from VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @param observationBuf
   ///     pointer to the buffer for observations
   /// @param bufLength
   ///     length of the buffer for observations
   /// @return
   ///     Return only number of observations if bufflength is 0 or observationBuf is a null pointer
   ///     otherwise return the number of obsrvations and fill the buffer with observations
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
   ///     Save a map to the given folder.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   /// @param mapFolder
   ///     Absolute path to the map folder. If this pointer is NULL, default folder in the engine is used.
   /// @param mapName
   ///     Name of Map. If this pointer is NULL, default name in the engine is used.
   /// @return
   ///     True if the map is successfully saved. False otherwise.
   //------------------------------------------------------------------------------
   bool RV_API rvVWSLAM_SaveMap( rvVWSLAM *pObj, const char* mapFolder, const char* mapName );


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Deinitialize VWSLAM object.
   /// @param pObj
   ///     Pointer to VWSLAM object.
   //------------------------------------------------------------------------------
   RV_API void rvVWSLAM_Deinitialize(rvVWSLAM *pObj);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Get map frame to odom frame transformation
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
