/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef RVWEF_H
#define RVWEF_H

/***************************************************************************//**
@file
   rvWEF.h

@detailed
   Robot Vision,
   Wheel Encoder Fusion (WEF)

@section Overview
   This feature takes a pose (i.e., location + orientation) from other RV
   features (e.g., VISLAM) along with the data from a robot’s wheel encoder HW
   and fuses the two together for a better and more reliable pose estimate.

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

#include <mvVSLAM.h>
#include "rv.h"

//==============================================================================
// Declarations
//==============================================================================

#ifdef __cplusplus
extern "C"
{
#endif


   enum ScaleVerificationStatus
   {
      ScaleVerificationOngoing = 0,
      ScaleVerificationPass,
      ScaleVerificationFail
   };


   struct ScaleVerification
   {
      int16_t failFrameNum4Verfi;
      int16_t verfiNum;

      float32_t scaleRatioThreshold;
      float32_t largeDistThreshold;
      float32_t smallDistThreshold;
      bool scaleEnable;

      int16_t failTimes = 0;
      int16_t passTimes = 0;

      bool isVerifiedSmall = false;
   };

   /************************************************************************//**
   @detailed
      Pose and velocity information with timestamp (microseconds).
   @param timestampUs
      Timestamp in microseconds.
   @param pose
      Pose in intrinsic Tait-Bryan format.
   @param velocityLinear
      Linear velocity along with X (heading) direction.
   @param velocityAngular
      Angular velocity along with Z (anti-clockwise) direction.
   ****************************************************************************/
   typedef struct
   {
      int64_t    timestamp;
      mvPose6DYPRT pose;
      float32_t  velocityLinear;
      float32_t  velocityAngular;
      MV_VSLAM_TRACKING_STATE poseQuality;  // Quality of the pose, same as the vSLAM pose quality
   } rvWEFPoseVelocityTime;


   /************************************************************************//**
   @detailed
      Pose information along with quality indicator and timestamp (microsecond).
   @param timestampUs
      Timestamp in microseconds.
   @param poseWithState
      Pose information along with quality indicator.
   ****************************************************************************/
   typedef struct
   {
      int64_t               timestamp;
      mvVSLAMTrackingPose   poseWithState;
   } rvWEFPoseStateTime;


   /************************************************************************//**
   @detailed
      Pose pair with time synchronized and outlier flag.
   @param pose
      Pose information along with quality indicator and timestamp (microsecond).
   @param wePose
      Pose and velocity information with timestamp (microseconds).
   @param isOutlier
      Exist outlier pose or not in this pair.
   ****************************************************************************/
   typedef struct
   {
      rvWEFPoseStateTime pose;
      rvWEFPoseVelocityTime wePose;
      bool isOutlier;
   } posePair;

   /************************************************************************//**
   @detailed
   Initialization mode of the world (map) frame.
   @param VWSALM_MODE
   World frame aligned with the first camera pose in vSLAM frame
   @param WHEEL_MODE
   World frame aligned with the first pose given out by wheel odometry
   ****************************************************************************/
   typedef enum
   {
      VSLAM_MODE = 0,
      WHEEL_MODE
   }rvWEFInitMode;


   /************************************************************************//**
   @detailed
      Wheel Encoder Fusion (WEF).
   ****************************************************************************/
   typedef struct rvWEF rvWEF;


   /************************************************************************//**
   @detailed
      Return string of version information.
   ****************************************************************************/
   //RV_API const char* rvWEF_Version( void );


   /************************************************************************//**
   @detailed
      Initialize WEF.
   @param poseVB
      Cross calibration parameters, pose of Body under Camera frame.
   @param loadMapFirst
      Load or build map first?  0: build; 1: load
   @param stateBadAsFail
      Treat state Bad as Fail?  1: yes; 0: no
   @param dofRestriction
      restrict DoF from 6 to 3? 1: yes; 0: no
   @returns
      Pointer to WEF object; returns NULL if failed.
   ****************************************************************************/
   RV_API rvWEF* rvWEF_Initialize( const mvPose6DYPRT* poseVB, 
                                   bool loadMapFirst, 
                                   bool stateBadAsFail, 
                                   bool dofRestriction );


   /************************************************************************//**
   @detailed
      Deinitialize WEF object.
   @param pObj
      Pointer to WEF object.
   ****************************************************************************/
   RV_API void rvWEF_Deinitialize( rvWEF* pObj );


   /************************************************************************//**
   @detailed
      Get pose of target image in world frame.
   @param pObj
      Pointer to WEF object.
   @param pose
      Pose of target image in world frame.
   @returns
      Successful or not.
   ****************************************************************************/
   RV_API bool rvWEF_GetTargetPose( rvWEF* pObj, mvPose6DYPRT& pose );


   /************************************************************************//**
   @detailed
      Set pose of target image in world frame.
   @param pObj
      Pointer to WEF object.
   @param pose
      Pose of target image in world frame.
   @returns
      Successful or not.
   ****************************************************************************/
   //RV_API bool rvWEF_SetTargetPose( rvWEF* pObj, mvPose6DET& pose );


   /************************************************************************//**
   @detailed
      Pass odometry from Wheel Encoder to the WEF object.
   @param pObj
      Pointer to WEF object.
   @param data
      Single odometry measurement data from Wheel Encoder.
   ****************************************************************************/
   RV_API void rvWEF_AddWheelOdom( rvWEF* pObj, const rvWEFPoseVelocityTime& data );


   /************************************************************************//**
   @detailed
      Pass pose to the WEF object.
   @param pObj
      Pointer to WEF object.
   @param poseWithState
      Single pose measurement data with quality indicator.
   @param timestampUs
      Timestamp of pose in microsecond.
   ****************************************************************************/
   RV_API void rvWEF_AddPose( rvWEF* pObj, mvVSLAMTrackingPose& poseWithState,
                              int64_t timestampUs );


   /************************************************************************//**
   @detailed
      Get pose from the WEF object.
   @param pObj
      Pointer to WEF object.
   @param data
      Single pose estimated from WEF.
   @returns
      If the estimated pose has been updated or not.
   ****************************************************************************/
   RV_API bool rvWEF_GetPose( rvWEF* pObj, rvWEFPoseVelocityTime& data );


   /************************************************************************//**
   @detailed
      Get corrected pose represented in World frame.
   @param pObj
      Pointer to WEF object.
   @param pose
      Single pose with quality indicator, represented in World frame.
   ****************************************************************************/
   RV_API void rvWEF_GetCorrectedPose( rvWEF* pObj, mvVSLAMTrackingPose& pose );


   /************************************************************************//**
   @detailed
      Recover pose represented in frame same as pose measurement data.
   @param pObj
      Pointer to WEF object.
   @param pose
      Single pose measurement data recovered.
   ****************************************************************************/
   RV_API bool rvWEF_RecoverPose( rvWEF* pObj, int64_t timestampUs, mvPose6DRT& pose );


   /************************************************************************//**
   @detailed
      Transform pose represented in Body frame to Camera frame.
   @param pObj
      Pointer to WEF object.
   @param pose6DET
      Single pose measurement represented in Body frame.
   @returns
      Single pose measurement represented in Camera frame.
   ****************************************************************************/
   RV_API mvPose6DRT rvWEF_BodyToCameraPose( rvWEF* pObj, 
                                             mvPose6DYPRT& pose6DET );


   /************************************************************************//**
   @detailed
      Set the initialization mode of world (map) frame in WEF
   @param pObj
      Pointer to WEF object.
   @param initMode
      WEF initialization mode
   ****************************************************************************/
   RV_API bool rvWEF_SetInitMode( rvWEF* pObj, rvWEFInitMode initMode );


   /************************************************************************//**
   @detailed
      Check if the WEF believe the pose from vslam or not
   @param pObj
      Pointer to WEF object.
   ****************************************************************************/
   RV_API bool rvWEF_TrackedOrNot( rvWEF* pObj );

   /************************************************************************//**
   @detailed
      Reset the WEF engine
   @param pObj
      Pointer to WEF object.
   ****************************************************************************/
   RV_API bool rvWEF_Reset( rvWEF* pObj );


   /************************************************************************//**
   @detailed
      Estimate scale of pose based on a vector of pose and Wheel data.
   @param poseQ
      Vector of pose.
   @param poseQLen
      Length of poseQ.
   @param wheelQ
      Vector of wheel data with real scale.
   @param wheelQLen
      Length of wheelQ.
   @param minDist2
      Square of minimal distance to compute scale.
   @param scale
      Estimated scale of pose.
   @param posePairQ
      Vector of pose pair with outlier rejected.
   @param posePairQLen
      Length of posePairQ.
   ****************************************************************************/
   RV_API bool rvWEF_EstimateScale( const rvWEFPoseStateTime* poseQ,
                                    unsigned int poseQLen,
                                    const rvWEFPoseVelocityTime* wheelQ,
                                    unsigned int wheelQLen,
                                    float minDist2,
                                    float32_t* scale, posePair* posePairQ,
                                    unsigned int* posePairQLen );


   /************************************************************************//**
   @detailed
   Estimate translation and rotation of the new VSLAM coordinate system in the old one,
   @param vslamPose
   Pose got from VSLAM in the new VSLAM coordinate at time k
   @param wePose
   Pose got from wheel at time k
   @param lastGoodvslamPose
   Pose got from VSLAM in the old VSLAM coordinate system at time 1
   @param lastwheelPose
   Pose got from wheel at time 1
   @param poseVB
   Pose of wheel in camera system
   @param scale
   Scale ratio between the new VSLAM coordinate system and the old one
   @param posePairQLen
   Pose of the new VSLAM coordinated system in the old one
   ****************************************************************************/
   RV_API bool rvWEF_EstimateTransform( const rvWEFPoseStateTime* vslamPose,
                                        const rvWEFPoseVelocityTime* wePose,
                                        const rvWEFPoseStateTime lastGoodvslamPose,
                                        const rvWEFPoseVelocityTime lastwheelPose,
                                        const float32_t* mPoseVBr,
                                        const float32_t* mPoseVBt,
                                        float32_t scale,
                                        float32_t* R, float32_t* t); 

   RV_API ScaleVerificationStatus rvWEF_VerifyScale( const rvWEFPoseStateTime* vslamPoseQ,
                                                     unsigned int vslamPoseQLen,
                                                     const rvWEFPoseVelocityTime* wePoseQ,
                                                     unsigned int wePoseQLen,
                                                     float32_t scale,
                                                     ScaleVerification &scaleVerificationV );

#ifdef __cplusplus
}
#endif


#endif
