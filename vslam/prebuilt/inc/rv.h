/*****************************************************************************
@copyright
Copyright (c) 2019-2023 Qualcomm Technologies, Inc.
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
#ifdef __ARM_NEON__
#include <arm_neon.h>
typedef float  float32_t;
typedef double float64_t;
#else
#include <stdint.h>
typedef float  float32_t;
typedef double float64_t;
#endif


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

   /***********************************
    * RVSDK OpenCL version requirement
    * And query OpenCL Info of the device
    * ********************************/
   RV_API void rvQueryOpenCLInfo( void );

   /************************************************************************//**
   @detailed
   
      The distortion model is limited to the following values :
      -\b NoDistortion = No distortion model\n
      - \b Polynomial4 = Four parameter polynomial[k1, k2, p1, p2] plumb - line( a.k.a.,
      Brown - Conrady ) model[D.C.Brown, "Photometric Engineering",
      Vol. 32, No. 3, pp.444 - 462 (1966)].Compatible with
      the oldest Caltech Matlab Calibration Toolbox.  To fill
      from OpenCV, declare cv::Mat for distortions with 5 rows( 1
      columns ), set it to zeros and use flag cv::CALIB_FIX_K3 with
      cv::calibrateCamera.\n
      - \b Polynomial5 = Five parameter polynomial[k1, k2, p1, p2, k3] plumb - line model.
      Compatible with current Matlab toolbox.To fill from OpenCV,
      declare cv::Mat for distortions with 5 rows, use flag
      cv::CALIB_FIX_K4 use cv::calibrateCamera.\n
      - \b RationalModel8 = Eight parameter rational polynomial( \i i.e.,
                                                    CV_CALIB_RATIONAL_MODEL )[k1, k2, p1, p2, k3, k4, k5, k6].\n
      - \b FisheyeModel4 = FishEye model[S.Shah, "Intrinsic Parameter Calibration
      Procedure for a( High - Distortion ) Fish - eye Lens Camera with
      Distortion Model and Accuracy Estimation"].  To fill from
      OpenCV, use cv::fisheye::calibrate.
   ****************************************************************************/
   typedef enum _DistortionModel
   {
      NoDistortion = 0,
      Polynomial4,
      Polynomial5,
      RationalModel8,
      FisheyeModel4
   } rvDistortionModel;

   /************************************************************************//**
   @detailed
      Camera calibration parameters.  This information could come from any
      calibration procedure.

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
      Distortion model as descriped in rvDistortionModel
   ****************************************************************************/
   typedef struct
   {
      // Image:
      uint32_t pixelWidth, pixelHeight, pixelStride;

      // Calibration:
      float32_t principalPoint[2];
      float32_t focalLength[2];
      float32_t distortion[8];
      rvDistortionModel   distortionModel;
   } rvCameraIntrinsic;

   /************************************************************************//**
   @detailed
      Stereo rig configuration.This information could come from any
      calibration procedure. The cameras are laid out in such a way
      as when looking from behind the cameras and into the direction that the
      camera points, the left camera is camera[0] and the right camera is
      camera[1].The camera coordinate systems are described in the
      mvCameraConfiguration description.

      The rig coordinate system is aligned with the camera[0] coordinate
      system.The positive x - axis is aligned with the camera[0] u - axis but
      would also be fairly close to the line between the centers of camera[0]
      and camera[1].This is the same coordinate system used by OpenCV.
   @param translation[3]
      Relative distance in meters added to a point from camera[1] in rig
      coordinates to align to the same point in camera[0].Therefore
      translation[0] is usually a negative number nearly equal to the baseline
      value since camera[1] is approximately the baseline value away along the
      rig coordinates x - axis.Same as self.T from ROS camera calibration tool
      and same as T from OpenCV cvStereoCalibrate() function.
      - translation[0] = x - axis translation.
      - translation[1] = y - axis translation.
      - translation[2] = z - axis translation( defined from the x - y plane ).
   @param rotation[3]
      Relative rotation vector between cameras.The rotation vector is a scaled
      axis - angle vector representation of the rotation between the two cameras
      also known as the Rodrigues' rotation formula in the aforementioned rig
      coordinate system.See https ://jsfiddle.net/1gej4qyp/ for example of
      converting a rotation matrix to scales - axis representation.Same as R
      from OpenCV cvStereoCalibrate() function.The ROS calibration tool output
      self.R would be the input rotation matrix to the Rodrigues' formula.
   @param camera[2]
      Left / right camera calibrations.
   @param correctionFactors[4]
      Polynomial coefficients for a distance - to - distance correction function.
      ****************************************************************************/
   typedef struct
   {
      float32_t translation[3], rotation[3];  // Relative between cameras
      rvCameraIntrinsic camera[2];            // Left/right camera calibrations
   } rvStereoCamera;


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
      6DRT type of pose with time stamp.
   @param pose
      Pose with Rotation-Translation matrix form
   @param timestamp
      time stamp with type of signed 64-bit integer
   ****************************************************************************/
   typedef struct
   {
       rvPose6DRT pose;
       int64_t timestamp;
   } rvPose6DRTWithTimestamp;

  /************************************************************************//**
   @detailed
      Pose information in Euler-Translation form.
      Euler angles in the Tait-Bryan ZYX intrinsic convention, unit is radian.
      For differences with rvPose6DET, refer to
      https://en.wikipedia.org/wiki/Euler_angles#Tait%E2%80%93Bryan_angles
   @param translation[3]
      Translation vector in meter.
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

   //unit: meter
   struct rvPoint3D
   {
      float32_t x;
      float32_t y;
      float32_t z;
   };


   struct rvPose6DQT
   {
      rvPoint3D robotPos;
      float32_t rx, ry, rz, rw;
   };



   /************************************************************************//**
   @detailed
      Position in 2D world space.
   ****************************************************************************/
   typedef struct
   {
       float32_t x; //unit: meter
       float32_t y; //unit: meter
   } rvPoint2D;


   /************************************************************************//**
   @detailed
      Position in 2D image coordinate.
   ****************************************************************************/
   typedef struct
   {
       int x; //unit: pixel
       int y; //unit: pixel
   } rvPixel2I;

   typedef struct
   {
       float32_t x; //unit: pixel
       float32_t y; //unit: pixel
   } rvPixel2F;

   /************************************************************************//**
   @detailed
      Region Of Interest
   @param x
      Left Up corner, X axis
   @param y
      Left Up corner, Y axis
   @param roiWidth
      Width of ROI
   @param roiHeight
      Height of ROI
   ****************************************************************************/
  typedef struct
  {
      int x;   //unit: pixel
      int y;   //unit: pixel
      int width;
      int height;
  } rvRoi2D;

#ifdef __cplusplus
}
#endif

#endif

