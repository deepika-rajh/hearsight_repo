/*****************************************************************************
@copyright
Copyright (c) 2019-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

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
#include <rvVSLAM.h>
//==============================================================================
// Declarations
//==============================================================================

#ifdef __cplusplus
extern "C"
{
#endif
      /************************************************************************//**
   @brief
      Pose information along with a quality indicator for VISLAM
   @param poseQuality
      Quality of the pose (no pose is provided if MV_TRACKING_STATE_INITIALIZING or MV_TRACKING_STATE_FAILED)
   @param bodyPose
      Body pose (in accelerometer frame, origin at time of first camera frame)
   @param gravityCameraPose
      Gravity aligned pose of camera
   @param errCovPose
      Error covariance matrix for pose
   @param timeAlignment
      Time misalignment for last frame (seconds)
   @param velocity
      Velocity in body frame
   @param errCovVelocity
      Error covariance for velocity
   @param angularVelocity
      Angular velocity in body frame
   @param gravity
      Gravity vector
   @param errCovGravity
      Error covariance for gravity
   @param wBias
      Gyro bias
   @param aBias
      Accelerometer bias
   @param rGyroBody
      Rotation of Gyro to body(body = accelerometer)
   @param aAccInv
      Accelerometer scale and non-orth
   @param aGyrInv
      Gyro scale and non-orth
   @param tbc
      Accelerometer-camera translational misalignment (meters)
   @param Rbc
      Accelerometer-camera rotational misalignment (rad), rotation matrix
   @param errorCode
      Error code (includes reasons for reset)
      bit 0: "Reset: cov not pos definite"
      bit 1: "Reset: IMU exceeded range"
      bit 2: "Reset: IMU bandwidth too low"
      bit 3: "Reset: not stationary at initialization"
      bit 4: "Reset: no features for x seconds"
      bit 5: "Reset: insufficient constraints from features"
      bit 6: "Reset: failed to add new features"
      bit 7: "Reset: exceeded instant velocity uncertainty"
      bit 8: "Reset: exceeded velocity uncertainty over window"
      bit 10: "dropped IMU samples"
      bit 11: "check intrinsic camera cal"
      bit 12: "insufficient number of good features to initialize"
   ****************************************************************************/
   struct rvVISLAMPose
   {
      RV_VSLAM_TRACKING_STATE poseQuality;
      rvPose6DRT bodyPose;
      rvPose6DRT gravityCameraPose;
      float32_t errCovPose[6][6];
      float32_t timeAlignment;
      float32_t velocity[3];
      float32_t errCovVelocity[3][3];
      float32_t angularVelocity[3];
      float32_t gravity[3];
      float32_t errCovGravity[3][3];
      float32_t wBias[3];
      float32_t aBias[3];
      float32_t rGyroBody[3][3];
      float32_t aAccInv[3][3];
      float32_t aGyrInv[3][3];
      float32_t tbc[3];
      float32_t Rbc[3][3];
      uint32_t errorCode;
      int64_t time;
   };

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
        rvCameraIntrinsic      *rvCameraCfg;  
        std::string            algConfigPath;
    }rvVIOCfg;


       /************************************************************************//**
   @brief
      Map point information from VISLAM
   @param id
      Unique ID for map point
   @param score
      Tracking score of map point
   @param pixLoc
      2D measured pixel location
   @param tsf
      3D location in spatial frame
   @param p_tsf
      Error covariance for tsf
   @param depth
      Depth of map point from camera
   @param depthErrorStdDev
      Depth error std dev
   @param robustWeight
      Robust weight of feature, expected range is 1-30. 1 is normal , if greater, 
      quality is less than normal.
   @param pointQuality
      Quality of the map point as per current VISLAM state.
   ****************************************************************************/
   struct rvVISLAMMapPoint
   {
      enum QUALITY_T
      {
         LOW,     ///< Uncertainty information not available for low quality points.
         MEDIUM,  ///< Points that are not tracked in state.
         HIGH     ///< Points that are in state
      };
      uint32_t  id;
      //uint32_t  score;
      float32_t pixLoc[2];
      float32_t tsf[3];
      float32_t p_tsf[3][3];
      float32_t depth;
      float32_t depthErrorStdDev;
      //float32_t robustWeight;
      QUALITY_T pointQuality;
   };

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
   //bool RV_API rvVIO_Execute(rvVIOHandle* pHandle, const std::string& sequenceDataDirectory);



   RV_API void rvVIO_AddImage( rvVIOHandle* pHandle, const int64_t timeStamp, const uint8_t *imageBuf );
   RV_API void rvVIO_AddAccel( rvVIOHandle* pHandle, int64_t time,
                               float64_t x, float64_t y, float64_t z );
   RV_API void rvVIO_AddGyro( rvVIOHandle* pHandle, int64_t time,
                              float64_t x, float64_t y, float64_t z );
   /// @detailed
   ///     Deinitialize VIO object.
   //------------------------------------------------------------------------------
   void RV_API rvVIO_Deinitialize(rvVIOHandle* pHandle);

   /************************************************************************//**
   @brief
      Grab last computed pose
   @param pObj
      Pointer to VISLAM object
   @return
      Computed pose from previous frame and IMU data
   ****************************************************************************/
   RV_API const rvVISLAMPose rvVIO_GetPose( rvVIOHandle* pHandle );


   /************************************************************************//**
   @brief
      Inquire if VISLAM has new map points.
   @param pObj
      Pointer to VISLAM object.
   @return
      Number of map points currently being observed and estimated 
   ****************************************************************************/
   int RV_API rvVIO_HasUpdatedPointCloud( rvVIOHandle* pHandle );


   /************************************************************************//**
   @brief
      Grab point cloud
   @param pObj
      Pointer to VISLAM object
   @param pPoints
      Pre-allocated array of mvVISLAMMapPoint struct to be filled in by VISLAM
      with current map points
   @param maxPoints
      Max number of points requested. Should match allocated size of pPoints.
   @return
      Number of points filled into the pPoints array
   ****************************************************************************/
   int RV_API rvVIO_GetPointCloud( rvVIOHandle* pHandle, rvVISLAMMapPoint* pPoints, uint32_t maxPoints );


   /************************************************************************//**
   @brief
      Resets the EKF from an external source. EKF will try to reinitialize in 
      the subsequent camera frame. To properly initialize after reset, device 
      should not be rotating, moving a lot, camera look at around 10 feats.  
      While calling mvVISLAM_Reset, it is recommended to preserve the IMU 
      samples between the frame when decision to reset is made and this API is
      called. So that the filter can make use of the IMU data at the time of 
      initialization.
   @param pObj
      Pointer to VISLAM object
   ****************************************************************************/
   void RV_API rvVIO_Reset( rvVIOHandle* pHandle );
   

#ifdef __cplusplus
}
#endif


#endif
