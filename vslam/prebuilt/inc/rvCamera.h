/*****************************************************************************
@copyright
Copyright (c) 2019-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef _ROBOT_VISION_CAMERA_H_
#define _ROBOT_VISION_CAMERA_H_
// @yanming move to src/
#include "rv.h"
#include <string>


/************************************************************************//**
@detailed
   The camera type is limited to the following values :
   -\b rvMonocular = Monocular camera\n
   -\b rvGrayDepth = Depth camera with gray images\n
   -\b rvStereo = Stereo camera\n
****************************************************************************/
typedef enum
{
    rvMonocular = 0,
    rvGrayDepth = 1,
    rvStereo
} rvCameraType;


/************************************************************************//**
@detailed
   The image format is limited to the following values:
   -\b Y_ONLY_FORMAT = Y only format\n
   -\b RAW_FORMAT = RAW format, 8 bits for each pixel\n
   -\b NV12_FORMAT = NV12 format
   -\b RBG_FORMAT = RGB format
****************************************************************************/
enum rvImageFormat
{
    Y_ONLY_FORMAT = 0,
    RAW_FORMAT,
    NV12_FORMAT,
    RGB_FORMAT,
};


/************************************************************************//**
@detailed
   Parameters of rectified and undistorted camera. Combined with original
   camera parameters, they can be used to generate 2 maps to warp
   rectified image from original image. They also can be used as camera intrinsic for
   localization and mapping when vSLAM engines use rectified images as input

@param pixelWidth
   Width of the rectified image in pixels.
@param pixelHeight
   Height of the rectified image in pixels.
@param P[3][4]
   Project matrix of the rectified camera, which projects a point in world coordinate to the camera's image.
@param R[3][3]
   Rotation matrix from rectified camera to original camera
@initialized
   Indicate if the parameters are set. If not, some functions to calculate
   these parameters have to be called. One way is to call OpenCV functions.
   For monocular cameras, R is usually 3*3 identity matrix and the first 3 columns of P
   can be got from cv::getOptimalNewCameraMatrix() and the 4th cloumn is set 0s.
   For stereo cameras, R and P can be got from initUndistortRectifyMap()
****************************************************************************/
typedef struct
{
    // Image:
    uint32_t pixelWidth, pixelHeight;

    // Calibration:
    double P[3][4];
    double R[3][3];
    bool initialized;
} rvRectCameraConfiguration;


/************************************************************************//**
@detailed
Parameters of rectified and undistorted stereo camera.
@param camera[2]
Rectified camera. camera[0] is the rectified left camera and
camera[1] is the rectified right camera.
@translation[3]
translation[0] is the rectified based line which is negative in general
****************************************************************************/
typedef struct
{
    float32_t translation[3];
    rvRectCameraConfiguration camera[2];
} rvStereoRectCamera;


/************************************************************************//*
@detailed
Data structur to support different cameras. 
@param imageFormat
The format of camera image
@param cameraType
The type of camera.
If the camera is monocular, only the stereo.camera[0] and stereoRec.camera[0] member are valid
if the camera is a depth camera, its rgb camera is usually rectifed 
and only the stereo.camera[0] is valid
if the camera is a stereo camera, both the stereo and stereoRect are valid

@param stereo
Configurations for stereo camera
@param stereoRect
Configuration for rectified stereo camera     
****************************************************************************/
struct rvCameraParams
{
   //for monocular camera, use the camera[0] in stereo and stereoRect
   rvCameraType cameraType;
   rvImageFormat imageFormat;
   rvStereoCamera stereo;
   rvStereoRectCamera stereoRect; //read from configuration file, if not invalide
};


/************************************************************************//**
@detailed
   IMU configuration.
@param imuEnabled
   if the IMU enabled. If not, input IMU measurements are invalid.
@param acceBias
   Bias of the accelormenter
@param gyroBias
   Bias of the gyroscope
@param deltaInSecond
   Time diff between the IMU clock and camera clock
@param cameraInIMU
   Camera pose in IMU coordinate
****************************************************************************/
struct IMUElement
{
    float32_t bias[3];
    float32_t matrix[3][3];
    float32_t noiseVariances[3];
    float32_t biasVariances[3];
};

typedef struct
{
    bool imuEnabled;
    IMUElement acc;
    IMUElement gyr;
    float32_t deltaInSecond;
    rvPose6DRT cameraInIMU;
} rvIMUConfiguration;


/************************************************************************//**
@detailed
   Wheel configuration.
@param wheelEnabled
   if the wheel enabled. If not, input wheel encode messages are invalid.
@param baselinkInCamera
   Pose of the wheel encoder in the camera  coordinate
****************************************************************************/
typedef struct
{
    bool wheelEnabled;
    rvPose6DRT baselinkInCamera; //also the cross-calibration matrix;
    float footprintSize; //the size of the baselink
} rvWheelConfiguration;


/************************************************************************//**
@detailed
   image size
@param width
   Width of the image in pixels
@param height
   Height of the image in pixels
@param stride
   Stride of the image in pixels
****************************************************************************/
typedef struct
{
   int             width;
   int             height;
   int             stride;
}rvImageSize;

/************************************************************************//**
@detailed
   Wheel configuration.
@param wheelEnabled
   if the wheel enabled. If not, input wheel encode messages are invalid.
@param baselinkInCamera
   Pose of the wheel encoder in the camera  coordinate
****************************************************************************/
typedef struct
{
    std::string path;
    float32_t targetWidth;
    float32_t targetHeight;
} rvTargetImage;

#endif
