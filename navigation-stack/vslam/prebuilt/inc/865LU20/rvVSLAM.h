/*****************************************************************************
@copyright
Copyright (c) 2019-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef RVVSLAM_H
#define RVVSLAM_H

/***************************************************************************//**
@file
   rvVSLAM.h

@detailed
   Robot Vision,
   Visual Simultaneous Localization And Mapping (VSLAM)
*******************************************************************************/


//==============================================================================
// Includes
//==============================================================================

#include <rv.h>
#include <rvCamera.h>

//==============================================================================
// Declarations
//==============================================================================

#ifdef __cplusplus
extern "C"
{
#endif

   
   //==============================================================================
   /// @detailed
   ///     Visual Simultaneous Localization And Mapping (VSLAM).
   //==============================================================================
   typedef struct rvVSLAM rvVSLAM;
   
   
   
   //------------------------------------------------------------------------------
   /// @detailed
   ///     Tracking observation.
   //------------------------------------------------------------------------------
   typedef struct _rvTrackedObservation
   {
      typedef enum
      {
         MATCHING_OK,                        ///< Matching succeeded
         MATCHING_FAILED                     ///< Matching failed
      } RV_OBSERVATION_STATE;
   
      float x = 0.f; //In pixel
      float y = 0.f; //in pixel
      int mapPointId = 0;
      RV_OBSERVATION_STATE s = RV_OBSERVATION_STATE::MATCHING_OK;
   } RV_TrackedObservation;
   
   
   //------------------------------------------------------------------------------
   /// @detailed
   ///     Active key frame.
   //------------------------------------------------------------------------------
   typedef struct
   {
      rvPose6DRT pose;
      int id;
      bool valid;
      bool free;
   } RV_ActiveKeyframe;


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Tracking state and quality for VSLAM.
   ///     which is decided by the number and accuracy of tracked map points
   //------------------------------------------------------------------------------
   typedef enum
   {
       RV_VSLAM_TRACKING_STATE_FAILED = -2,
       RV_VSLAM_TRACKING_STATE_INITIALIZING = -1,
       RV_VSLAM_TRACKING_STATE_GREAT = 0,
       RV_VSLAM_TRACKING_STATE_GOOD = 1,
       RV_VSLAM_TRACKING_STATE_OK = 2,
       RV_VSLAM_TRACKING_STATE_BAD = 3,
       RV_VSLAM_TRACKING_STATE_APPROX = 4,
   } RV_VSLAM_TRACKING_STATE;
   
   
   //------------------------------------------------------------------------------
   /// @detailed
   ///     Pose information along with a quality indicator for VSLAM.
   //------------------------------------------------------------------------------
   typedef struct
   {
      rvPose6DYPRT pose;                    // Pose                
      RV_VSLAM_TRACKING_STATE poseQuality;  // Quality of the pose
      int32_t    coordinateId;              // The Id of the coordinate for the pose
      uint64_t   timestampNs;               // Timestamp in nanosecond
   } rvVSLAMPose;

  
   typedef enum
   {   
      RV_TARGET_INIT = 0,
      RV_TARGETLESS_INIT = 1,
      RV_RELOCALIZATION = 2,
      RV_DEPTH_INIT = 3,
      RV_VSLAM_ONLY = 4,
      RV_RELOCALIZATION_DEPTH = 5,
      RV_NONE_INIT = -1
   } rvVSLAM_INITMODE;
   
   
   //------------------------------------------------------------------------------
   /// @detailed
   ///     Initialize VSLAM object.
   /// @param pnConfig
   ///     Pointer to VSLAM configuration.
   /// @param objectName
   ///     Name of the VSLAM object.
   /// @return
   ///     Pointer to VSLAM object; returns NULL if failed.
   //------------------------------------------------------------------------------
   RV_API rvVSLAM* rvVSLAM_Initialize( const rvCameraIntrinsic *pnConfig, 
                                       const rvWheelConfiguration *wheelConfig,
                                       const rvIMUConfiguration *imuConfig,
                                       const char *objectName );
   
   
   //------------------------------------------------------------------------------
   /// @detailed
   ///     Deinitialize VSLAM object.
   /// @param pObj
   ///     Pointer to VSLAM object.
   //------------------------------------------------------------------------------
   void RV_API rvVSLAM_Deinitialize( rvVSLAM* pObj );
   
   
   //------------------------------------------------------------------------------
   /// @detailed
   ///     Pass camera frame to the VSLAM object.
   /// @param pObj
   ///     Pointer to VSLAM object.
   /// @param t
   ///     Timestamp of camera frame.
   /// @param pxls
   ///     Pointer to camera frame data.
   /// @param pPriorPose
   ///     Reference robot odometry in VSLAM coordinate system.
   //------------------------------------------------------------------------------
   void RV_API rvVSLAM_AddImage( rvVSLAM* pObj, int64_t t, const uint8_t* pxls, const rvPose6DRT * pPriorPose, int coordinateId );
   
   
   //------------------------------------------------------------------------------
   /// @detailed
   ///     Pass camera frame and depth image to the VSLAM object.
   /// @param pObj
   ///     Pointer to VSLAM object.
   /// @param t
   ///     Timestamp of camera frame.
   /// @param pxls
   ///     Pointer to camera frame data.
   /// @param depth
   ///     Pointer to depth image data.
   /// @param pPriorPose
   ///     Reference robot odometry in VSLAM coordinate system.
   //------------------------------------------------------------------------------
   void RV_API rvVSLAM_AddImageDepth(rvVSLAM* pObj, int64_t t, const uint8_t* pxls, const uint16_t* depth, const rvPose6DRT * pPriorPose, int coordinateId);
   
   
   //------------------------------------------------------------------------------
   /// @detailed
   ///     Compute and return pose.
   /// @param pObj
   ///     Pointer to VSLAM object.
   /// @return
   ///     Computed pose from previous frame and IMU data.
   //------------------------------------------------------------------------------
   const rvVSLAMPose RV_API rvVSLAM_GetPose( rvVSLAM* pObj );
   
   //------------------------------------------------------------------------------
   /// @detailed
   ///     Add target to VSLAM internal target database.
   /// @param pObj
   ///     VSLAM object.
   /// @param name
   ///     Target name.
   /// @param pxls
   ///     Pointer to image.
   /// @param pxlWidth
   ///     Image width in pixels.
   /// @param pxlHeight
   ///     Image height in pixels.
   /// @pxlStride
   ///     Image memory stride.
   /// @param targetWidth
   ///     Physical width of target.
   /// @param targetHeight
   ///     Physical height of target.
   /// @param targetPose
   ///     6DOF pose of target ( center and rotation ).
   /// @return
   ///     On success target ID >= 0
   ///     -1 on failure
   //------------------------------------------------------------------------------
   int RV_API rvVSLAM_AddTarget( rvVSLAM* pObj, const char* name, 
                                 const uint8_t* pxls, uint32_t pxlWidth, 
                                 uint32_t pxlHeight, uint32_t pxlStride, 
                                 float32_t targetWidth, float32_t targetHeight, 
                                 rvPose6DRT targetPose );
   
   
   //------------------------------------------------------------------------------
   /// @detailed
   ///     Inquire if VSLAM has new map points.
   /// @param pObj
   ///     Pointer to VSLAM object.
   /// @return
   ///     Number of current map points.
   //------------------------------------------------------------------------------
   int RV_API rvVSLAM_HasUpdatedPointCloud( rvVSLAM *pObj );
   
   
   //------------------------------------------------------------------------------
   /// @detailed
   ///     Grab point cloud.
   /// @param pObj
   ///     Pointer to VSLAM object.
   /// @param pPoints
   ///     Pre-allocated array of 3 floats per map point queried.
   /// @param maxPoints
   ///     Max number of points requested. Should match allocated number of 
   ///     points.
   /// @return
   ///     Number of points filled into the pPoints array (number of triples). 
   ///     This can be smaller then number returned by 
   ///     rvVSLAM_HasUpdatedPointCloud.
   //------------------------------------------------------------------------------
   int RV_API rvVSLAM_GetPointCloud( rvVSLAM* pObj, float* pPoints, 
                                     uint32_t maxPoints );
   
   
   //------------------------------------------------------------------------------
   /// @detailed
   ///     Get the number of key frames in the maps.
   /// @param pObj
   ///     Pointer to VSLAM object.
   /// @return
   ///     Number of key frames in the first source map (there might be multiple 
   ///     source maps in SLAM).
   //------------------------------------------------------------------------------
   int RV_API rvVSLAM_GetMapSize( rvVSLAM* pObj );
   
   
   //------------------------------------------------------------------------------
   /// @detailed
   ///     Save a map to the given path.
   /// @param pObj
   ///     Pointer to VSLAM object.
   /// @param mapFolder
   ///     The folder for saving map.
   /// @param mapName
   ///     Name of Map.
   /// @return
   ///     True if the map is successfully saved. False otherwise.
   //------------------------------------------------------------------------------
   bool RV_API rvVSLAM_SaveMap( rvVSLAM* pObj, const char* mapFolder, 
                                const char* mapName );
   
   
   //------------------------------------------------------------------------------
   /// @detailed
   ///     Preset the path to load a startup map for SLAM.
   /// @param pObj
   ///     Pointer to VSLAM object.
   /// @param mapPath
   ///     The map path.
   /// @param addingKeyframesEnabled
   ///     Whether to enable to add new key frames to the initial map.
   //------------------------------------------------------------------------------
   void RV_API rvVSLAM_SetMapPathAndName( rvVSLAM* pObj, const char *mapPath, const char * mapName );
   
   
   
   //------------------------------------------------------------------------------
   /// @detailed
   ///     Reset all map and pose, then start from zero state.
   /// @param pObj
   ///     Pointer to VSLAM object.
   /// @param bool
   ///     bool value for backup the map or not
   /// @param mapBackup
   ///     Pointer to map backup
   //------------------------------------------------------------------------------
   void RV_API rvVSLAM_DeepReset( rvVSLAM* pObj, bool saveMap, void * mapBackup );
   
   
   //------------------------------------------------------------------------------
   /// @detailed
   ///     Override target initialization from rvVSLAM_AddTarget() with scale-free
   ///     tracking instead.
   /// @param pObj
   ///     Pointer to VSLAM object.
   /// @param enable
   ///     TRUE = turn on scale-free tracking and ignores initialization target.
   ///     FALSE = turn off scale-free tracking and return to target 
   ///             initialization.
   //------------------------------------------------------------------------------
   void RV_API rvVSLAM_SetInitMode( rvVSLAM* pObj, rvVSLAM_INITMODE mode );
   
   
#ifdef __cplusplus
}
#endif


#endif
