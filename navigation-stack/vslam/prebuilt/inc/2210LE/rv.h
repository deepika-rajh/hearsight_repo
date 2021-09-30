/*****************************************************************************
@copyright
Copyright (c) 2021 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/


#ifndef RV_H
#define RV_H
/***************************************************************************//**
@file
   rv.h

@detailed
   Common data structures and utilities for the Robot Vision SDK.

@mainpage
   Robot Vision SDK

@version
   1.0.0 (WIP1)


@section Overview
   QTI's Robot Vision SDK provides highly runtime optimized and state of
   the art robot vision algorithms to enable such features as localization,
   and mapping.  Some example features included are:
   - Visual Simultaneous Localization and Mapping (VSLAM) for robot
     localization and pose estimation.
   - Voxel Map (VM) for 3D depth fusion and mapping.

*******************************************************************************/
#include <stddef.h>
#include <stdbool.h>
#include <vector>

#ifdef __GNUC__
#ifdef BUILDING_SO
// MACRO enables function to be visible in shared-library case.
#define RV_API __attribute__ ((visibility ("default")))
#else
// MACRO empty for non-shared-library case.
#define RV_API
#endif
#else

#ifdef WIN32
// MACRO enables function to be visible in shared-library case.
#define RV_API __declspec(dllexport)
#else
// MACRO empty for non-shared-library case.
#define RV_API
#endif
#endif

//==============================================================================
// Includes
//==============================================================================

#include <stddef.h>
#include <stdbool.h>

#ifdef __ARM_NEON__
#include <arm_neon.h>
typedef float  float32_t;
typedef double float64_t;
#else
#include <stdint.h>
typedef float  float32_t;
typedef double float64_t;
#endif

//==============================================================================
// Declarations
//==============================================================================

#ifdef __cplusplus
extern "C"
{
#endif

   /***********************************
    * return RV SDK version string
    * ********************************/
   RV_API const char* rvVersion( void );

   struct PointAE {
       double x;
       double y;
       double z;
   };


   struct AEPOSE {
       PointAE robotPos;
       double rx, ry, rz, rw;
   };


   struct Frontier {
       uint32_t size;
       double min_distance;
       double cost;
       PointAE initial;
       PointAE centroid;
       PointAE middle;
       std::vector<PointAE> points;
   };


   struct MapInfo {
       unsigned char* map_;
       unsigned int width_;
       unsigned int height_;
       float mapResolution_;
       float mapOriginX_;
       float mapOriginY_;
   };


   enum STATUS { PURSUEGOAL, BACKTOORIGIN, ROTATE, AGAIN, HOLDON };


   /************************************************************************//**
   @brief
      Preference of tradeoffs (e.g., speed vs quality)
   ****************************************************************************/
  /* typedef enum
   {
      MV_MODE_SPEED = 0,
      MV_MODE_QUALITY = 1,
      MV_MODE_GPU = 2,
      MV_MODE_GPU_SIM = 3
   } MV_MODE;*/

   /************************************************************************//**
   @detailed
      Camera calibration parameters.  This information could come from any
      calibration procedure including the CAC feature within this library.

      The pixel coordinate space [u, v] has the origin [0, 0] in the upper-left
      image corner.  The u-axis runs towards right along the row in memory
      address increasing order, and the v-axis runs downward along the column
      also in memory address increasing order but with a stride length equal to
      the row width.

      The camera coordinate system [x, y, z] is centered on the camera principle
      point.  The positive x-axis of the camera points from the center principle
      point along that row of pixels [u].  The y-axis points down from the
      camera center along a column of pixels [v].  The z-axis points directly
      out along the optical axis in the direction that the camera is pointing.

      \b NOTE:  This is the same coordinate system used by OpenCV.
   @param pixelWidth
      Width of the image in pixels.
   @param pixelHeight
      Height of the image in pixels.
   @param memoryStride
      Memory width in bytes to the same pixel one row below.
   @param uvOffset
      Optional memory offset to UV plane for NV21 images.  Note, this is the
      U and V color planes of the NV21 format and not to be confused with
      the u and v axes in image space.
   @param principalPoint[2]
      Principal point [u, v] in pixels is defined relative to camera origin
      in pixel space where [0, 0] is the upper-left image corner, u runs
      towards right along the row, and v runs downward along the column.
   @param focalLength[2]
      Focal length expressed in pixels and as separate components along the
      image [width, height].  These components are aligned with the [u, v] axes
      of the principalPoint[2].
   @param distortion
      Distortion coefficients.  All unused array elements must be set to 0.
      distortion[0] would be equivalent to k1 in OpenCV or the constant a in
      the fisheye paper, distortion[1] would be k2 or the constant b in the
      paper, and so on.
   @param distortionModel
      The distortion model is limited to the following values:
      - \b 0 = No distortion model\n
      - \b 4 = Four parameter polynomial [k1, k2, p1, p2] plumb-line (a.k.a.,
               Brown-Conrady) model [D. C. Brown, "Photometric Engineering",
               Vol. 32, No. 3, pp.444-462 (1966)].  Compatible with
               the oldest Caltech Matlab Calibration Toolbox To fill
               from OpenCV, declare cv::Mat for distortions with 5 rows (1
               columns), set it to zeros and use flag cv::CALIB_FIX_K3 with
               cv::calibrateCamera.\n
      - \b 5 = Five parameter polynomial [k1, k2, p1, p2, k3] plumb-line model.
               Compatible with current Matlab toolbox.  To fill from OpenCV,
               declare cv::Mat for distortions with 5 rows, use flag
               cv::CALIB_FIX_K4 use cv::calibrateCamera.\n
      - \b 8 = Eight parameter rational polynomial (\i i.e.,
               CV_CALIB_RATIONAL_MODEL) [k1, k2, p1, p2, k3, k4, k5, k6].\n
      - \b 10 = FishEye model [S.Shah, "Intrinsic Parameter Calibration
                Procedure for a (High-Distortion) Fish-eye Lens Camera with
                Distortion Model and Accuracy Estimation"].  To fill from
                OpenCV, use cv::fisheye::calibrate.
   ****************************************************************************/
   typedef struct
   {
      // Image:
      uint32_t pixelWidth, pixelHeight;

      // Image Memory:
      uint32_t memoryStride;
      uint32_t uvOffset;

      // Calibration:
      float64_t principalPoint[2];
      float64_t focalLength[2];
      float64_t distortion[8];
      int32_t   distortionModel;
   } rvCameraConfiguration;


/************************************************************************//**
   @brief
      Configuration of stereo config.
   ****************************************************************************/
   typedef struct
   {
      float32_t translation[3], rotation[3];  // Relative between cameras
      rvCameraConfiguration camera[2];        // Left/right camera calibrations
      float32_t correctionFactors[4];         // Distance correction
   } rvStereoConfiguration;
   /************************************************************************//**
   @detailed
      6-DOF pose information in Rotation-Translation matrix form.
   @param matrix
      [ R | T ] rotation matrix + translation column vector in row major order.
   ****************************************************************************/
   typedef struct
   {
      float32_t matrix[3][4];
   } rvPose6DRT;

  /************************************************************************//**
   @detailed
   Pose information in Euler-Translation form.
   Euler angles in the Tait-Bryan ZYX intrinsic convention, unit is radian.
   For differences with rvPose6DET, refer to
   https://en.wikipedia.org/wiki/Euler_angles#Tait%E2%80%93Bryan_angles
   @param translation[3]
   Translation vector in use defined units.
   @param euler[3]
   Euler angles in the Tait-Bryan ZYX intrinsic convention, unit is in radian.
   \n euler[0] = rotation about x-axis. roll
   \n euler[1] = rotation about y-axis. pitch
   \n euler[2] = rotation about z-axis yaw.
    ****************************************************************************/
   typedef struct
   {
      float32_t translation[3];
      float32_t euler[3];
   } rvPose6DYPRT;

   /************************************************************************//**
   @detailed
      Pose information in Euler-Translation form.
      Euler angles in the Tait-Bryan ZYX extrinsic convention. unit is degree
      For differences with rvPose6DET, refer to
      https://en.wikipedia.org/wiki/Euler_angles#Tait%E2%80%93Bryan_angles
   @param translation[3]
      Translation vector in use defined units.
   @param euler[3]
      Euler angles in the Tait-Bryan ZYX extrinsic convention. unit is degree
      \n euler[0] = rotation about x-axis.
      \n euler[1] = rotation about y-axis.
      \n euler[2] = rotation about z-axis (defined from y-axis).
   ****************************************************************************/
   /*typedef struct
   {
      float32_t translation[3];
      float32_t euler[3];
   } rvPose6DET;

   /************************************************************************//**
   @detailed
      Multiply two mvPose6DRT, computes out = A * B
   ****************************************************************************/
   //RV_API void rvMultiplyPose6DRT( const rvPose6DRT* A, const rvPose6DRT* B, rvPose6DRT* out );

   /************************************************************************//**
   @detailed
      Position in 2D world space.
   ****************************************************************************/
   typedef struct
   {
       float32_t x; //unit: meter
       float32_t y; //unit: meter
   } rvPosition2D;


   /************************************************************************//**
   @detailed
      Position in 2D image coordinate.
   ****************************************************************************/
   typedef struct
   {
       int x; //unit: pixel
       int y; //unit: pixel
   } rvPixel2D;



   //------------------------------------------------------------------------------
   /// @detailed
   ///     Path planner configurable parameters.
   //------------------------------------------------------------------------------
   //------------------------------------------------------------------------------
   typedef struct
   {
       float resolution; //map resolution       
   } rvPathPlanningParameters;


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Tracking state quality for VSLAM.
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
   ///     Grip map state for PLANNER.
   //------------------------------------------------------------------------------
   typedef enum
   {
       RV_PLANNER_FRIDMAP_FREE = 0,
       RV_PLANNER_FRIDMAP_UNKNOWN = 255,
       RV_PLANNER_FRIDMAP_OCCUPIED = 254
   } RV_PLANNER_GRIDMAP_STATE;


#ifdef __cplusplus
}
#endif

#ifdef __ARM_NEON__
#include <arm_neon.h>
typedef float  float32_t;
typedef double float64_t;
#else
#include <stdint.h>
typedef float  float32_t;
typedef double float64_t;
#endif


#endif

