/*****************************************************************************
@copyright
Copyright (c) 2017-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef MVSRW_H
#define MVSRW_H

/***************************************************************************//**
@file
   mvSRW.h

@brief
   Sequence Reader Writer (SRW)

@defgroup mvsrw Sequence Reader Writer

@section Overview
   The SRW feature is for reading and writing data sequences that can be inputs
   into other MV features.  One work flow might be to capture several cameras
   and IMU data using mvCapture which will write out a SRW sequence.  That
   sequence can then be fed into a MV playback tool (e.g., mvDFSPlayback).

   The sequences are saved as a directory structure of files.  The directory
   structure needs to be the following:

   \code
   data/
     accelerometer.xml
     attitude.xml
     cameraSettings.xml
     gyroscope.xml
     Configuration.VIO.playback.XML
   data/Camera
     frame_00000.pgm
     . . .
     MetaInfo.xml\endcode

   This directory and the contents is created by the Writer but the xml file
   describing the data (e.g., Configuration.VIO.playback.XML in this case) can
   be corrupted.  It can be created by a user and placed in the data directory
   by hand.

   The example config file looks like the following:

   \code
   <?xml version='1.0' encoding='utf-8'?>
   <Configuration>
     <Offline>
        <Camera folder="./Camera/" framerate="WAIT" loop="false" />
         <Sensor folder="./" loop="false" />
      </Offline>
   </Configuration>\endcode

@section Limitations
   The following list are some of the known limitations:

   - Writer object must be properly de-initialized for file writing to
     complete.
   - All data except images must fit into application RAM.  However, if data is
     written faster than the disk write speed then all data including images
     must fit into memory.
*******************************************************************************/


//==============================================================================
// Defines
//==============================================================================


//==============================================================================
// Includes
//==============================================================================

#include <mv.h>


#ifdef __cplusplus
extern "C"
{
#endif


/***************************************************************************//**
@brief
   Image data structure.

@param pixels
   Pointer to 8-bit grayscale image luminance data.

@param width
   Width of image in pixels.

@param height
   Height of image in pixels.

@param memoryStride
   Number of bytes to pixel directly one row below.

@ingroup mvsrw
*******************************************************************************/
struct mvImage
{
   // Image data:
   uint8_t* pixels;

   uint32_t width;
   uint32_t height;

   // Image Memory:
   uint32_t memoryStride;
};



/***************************************************************************//**
@brief
   Image data structure.

@param pixels
   Pointer to 16-bit grayscale image luminance data.

@param width
   Width of image in pixels.

@param height
   Height of image in pixels.

@param memoryStride
   Offset to pixel directly one row below.

@ingroup mvsrw
*******************************************************************************/
struct mvImage16
{
	// Image data:
	uint16_t* pixels;

	uint32_t width;
	uint32_t height;

	// Image Memory:
	uint32_t memoryStride;
};



/***************************************************************************//**
@brief
   Camera frame.

@param timestamp
   Time stamp of data in nanoseconds.  Time must be center of exposure time
   and not the start or end of frame.

@param leftImage
   This is the only image in the monocular case.  In the stereo case, this
   is the left image.

@param rightImage
   In the stereo case, this is the right image.  In the monocular case, it
   is invalid.

@param depthImage
	In the depth case, this is the depth image.  In the monocular case, it
	is invalid.

@ingroup mvsrw
*******************************************************************************/
struct mvFrame
{
   char cameraName[256];

   // Time stamp
   int64_t timestamp;

   mvImage* leftImage;
   mvImage* rightImage;
	mvImage16* depthImage;
};



/***************************************************************************//**
@brief
   IMU data structure.

@param timestamp
   Time stamp of data in nanoseconds.

@param x
   Value for the x-axis.

@param y
   Value for the y-axis.

@param z
   Value for the z-axis.

@ingroup mvsrw
*******************************************************************************/
struct mvIMUData
{
   // Time stamp
   int64_t timestamp;

   // IMU data
   float64_t x;
   float64_t y;
   float64_t z;
};



/***************************************************************************//**
@brief
   GPS time sync data.

@param timestamp
   Time stamp of data in nanoseconds.

@param bias
   Value for the time bias/offset between GPS and IMU (system) clocks.

@param GPStimeUncertaintyStd
   GPS time uncertainty.

@ingroup mvsrw
*******************************************************************************/
struct mvGPStimeSyncData
{
      // Time stamp
      int64_t timestamp;

       int64_t bias;
       int64_t drift;
       int64_t GPStimeUncertaintyStd;
   };



/***************************************************************************//**
@brief
   GPS velocity data.

@param timestamp
   GPS Time stamp of data in picoseconds.

@param x
   Velocity for the x-axis.

@param y
   Velocity for the y-axis.

@param z
   Velocity for the z-axis.

@ingroup mvsrw
*******************************************************************************/
struct mvGPSvelocityData
{
      // Time stamp
      int64_t timestamp;

      float64_t x;
      float64_t y;
      float64_t z;
      float64_t measErrorCov[3][3];
      uint16_t solutionInfo;
};



/***************************************************************************//**
@brief
   Attitude estimate.

@param timestamp
   Time stamp of data in nanoseconds.

@param rotation_matrix
   World to body rotation matrix (R) in row major order.  Example:
   \code
   a0 = [0 0 g]
      a = R^T * a0
      a = [-sin(pitch)
            cos(pitch) * sin(roll)
            cos(pitch) * cos(roll)] * g
   \endcode
   where pitch, roll, and yaw are using Tait-Bryan ZYX convention and yaw
   from magnetic north.

@ingroup mvsrw
*******************************************************************************/
struct mvAttitudeData
{
   enum
   {
      ATTITUDE_MAT_SIZE = 9
   };
   // Time stamp
   int64_t timestamp;

      // IMU data
      float32_t rotation_matrix[ATTITUDE_MAT_SIZE];
   };


   typedef enum
   {
      mvMonocular = 0,
      mvGrayDepth = 1,
      mvStereo
   } mvCameraType;



   struct mvCameraDescriptor
   {
      char name[256];    //name of the camera based on Configuration
      mvCameraType type;
   };

/***************************************************************************//**
@param desc
   Camera descriptor to be used as correspondence with camera name given by
   frames.

@param params
   Camera parameters.

@ingroup mvsrw
*******************************************************************************/
struct mvCameraData
{
   mvCameraDescriptor desc;
   mvCameraConfiguration params;
};

   struct mvCameraInit
   {
      const char* name;
      int width, height;
      mvCameraType type;
      bool grayImage;
   };


   /************************************************************************//**
   @param rbc
      Rotation from camera coordinate to body coordinate use by attitude.
   @param timeOffset
      Offset between camera and IMU timestamps. IMU timestamp translates
      to camera timestamp t + timeOffset.
   @param rollingShutterSkew
      Rolling shutter skew of the camera, which is the elapsed time from
      beginning of the first image row to the beginning of the last row.
   ****************************************************************************/
   struct mvCameraExtrinsicParameters
   {
      mvPose3DR rbc;
      int64_t timeOffset;
      int64_t rollingShutterSkew;
   };



/***************************************************************************//**
@brief
   Sequence Writer for IMU and camera data.
*******************************************************************************/
typedef struct mvSRW_Writer mvSRW_Writer;



/***************************************************************************//**
@brief
   Initialize SequenceWriter object.

@param folderPath
   Location on storage where to save the sequence files.

@param monoCam
   Pointer to monocular camera object.

@param stereoCam
   Pointer to stereo camera object.

@return
   Pointer to SequenceWriter object; returns NULL if failed.

@ingroup mvsrw
*******************************************************************************/
MV_API mvSRW_Writer* mvSRW_Writer_Initialize( const char* folderPath, mvCameraInit* camera);



/***************************************************************************//**
@brief
   De-initialize SequenceWriter object.

@param pObj
   Pointer to SequenceWriter object.

@ingroup mvsrw
*******************************************************************************/
void MV_API mvSRW_Writer_Deinitialize( mvSRW_Writer* pObj );



/***************************************************************************//**
@brief
   Pass camera frame to the MV SequenceWriter object.

@param pObj
   Pointer to SequenceWriter object.

@param time
   Time stamp of camera frame.

@param pxls
   Pointer to camera frame data.

@ingroup mvsrw
*******************************************************************************/
void MV_API mvSRW_Writer_AddImage( mvSRW_Writer* pObj, int64_t time,
                                      const uint8_t* pxls, const uint16_t * depth );



/***************************************************************************//**
@brief
   Pass stereo camera frame to the MV SequenceWriter object.

@param pObj
   Pointer to SequenceWriter object.

@param time
   Time stamp of camera frame.

@param pxlsL
   Pointer to left camera frame data.

@param pxlsR
   Pointer to right camera frame data.

@ingroup mvsrw
*******************************************************************************/
void MV_API mvSRW_Writer_AddStereoImage( mvSRW_Writer* pObj, int64_t time,
                                          const uint8_t* pxlsL,
                                          const uint8_t* pxlsR );



/***************************************************************************//**
@brief
   Pass Accelerometer data to the SequenceWriter object.

@param pObj
   Pointer to SequenceWriter object.

@param time
   Time stamp of accelerometer data.

@param x
   Accelerometer data for X axis.

@param y
   Accelerometer data for Y axis.

@param z
   Accelerometer data for Z axis.

@ingroup mvsrw
*******************************************************************************/
void MV_API mvSRW_Writer_AddAccel( mvSRW_Writer* pObj, int64_t time,
                                    float64_t x, float64_t y, float64_t z );



/***************************************************************************//**
@brief
   Pass Gyroscope data to the SequenceWriter object.

@param pObj
   Pointer to SequenceWriter object.

@param time
   Time stamp of Gyro data.

@param x
   Gyro data for X axis.

@param y
   Gyro data for Y axis.

@param z
   Gyro data for Z axis.

@ingroup mvsrw
*******************************************************************************/
void MV_API mvSRW_Writer_AddGyro( mvSRW_Writer* pObj, int64_t time,
                                    float64_t x, float64_t y, float64_t z );



/***************************************************************************//**
@brief
   Pass GPS time sync data to the SequenceWriter object.

@param pObj
   Pointer to SequenceWriter object.

@param time
   Time stamp of data in system time in nanoseconds.

@param bias
   Time bias/offset (time bias/offset = GPS time - system time) in
   nanoseconds.

@param drift
   Drift of system time w.r.t. GPS time (not currently used).

@param GPStimeUncertaintyStd
   GPS time estimation uncertainty (set to -1 if not available).

@ingroup mvsrw
*******************************************************************************/
void MV_API mvSRW_Writer_AddGpsTimeSync( mvSRW_Writer* pObj, int64_t time,
      int64_t bias, int64_t drift, int64_t GPStimeUncertaintyStd );



/***************************************************************************//**
@brief
   Pass GPS velocity data to the SequenceWriter object.

@param pObj
   Pointer to SequenceWriter object.

@param time
   Time stamp of data in GPS time in nanoseconds.

@param x
   GPS velocity in East direction in m/s.

@param y
   GPS velocity in North direction in m/s.

@param z
   GPS velocity in Up direction in m/s.

@param xStd
   Standard deviation of velocity uncertainty in East in m/s.

@param yStd
   Standard deviation of velocity uncertainty in North in m/s.

@param zStd
   Standard deviation of velocity uncertainty in Up in m/s.

@param solutionInfo
   Fix type/quality: the last 3 bits being '100' represents a good message
   (if available, otherwise set to 4).

@ingroup mvsrw
*******************************************************************************/
void MV_API mvSRW_Writer_AddGpsVelocity( mvSRW_Writer* pObj, int64_t time,
      float64_t x, float64_t y, float64_t z, float64_t xStd, float64_t yStd,
                                          float64_t zStd,
                                          uint16_t solutionInfo );



/***************************************************************************//**
@brief
   Pass CameraSettings data to the SequenceWriter object.

@param pObj
   Pointer to SequenceWriter object.

@param time
   Time stamp of CameraSettings data.

@param gain
   Gain settings applied to the camera.

@param exposure
   Exposure time applied to the camera.

@ingroup mvsrw
*******************************************************************************/
void MV_API mvSRW_Writer_AddCameraSettings( mvSRW_Writer* pObj, int64_t time,
                                             float64_t gain,
                                             float64_t exposure,
                                             float64_t exposureScaled );



/***************************************************************************//**
@brief
   Pass Attitude data to the SequenceWriter object.

@param pObj
   Pointer to SequenceWriter object.

@param time
   Pointer to the mvAttitudeData array.

@param numAttitudes
   Size for the above array.

@ingroup mvsrw
*******************************************************************************/
void MV_API mvSRW_Writer_AddAttitude( mvSRW_Writer* pObj,
                                      mvAttitudeData* mvAttitudeDataPtr,
                                      int32_t numAttitudes );



/***************************************************************************//**
@brief
   Write file with name \<name\>.cal with camera parameters.

@param pObj
   Pointer to SequenceWriter object.

@param name
   Camera name, used for filename and should be same as in initialization.

@param config
   Camera parameters to be written.

@ingroup mvsrw
*******************************************************************************/
void MV_API mvSRW_Writer_AddCameraParameters( mvSRW_Writer* pObj,
                                              const char* name,
                                              mvCameraConfiguration* config );


   void MV_API mvSRW_Writer_AddColorImage( mvSRW_Writer* pObj, int64_t time, const uint8_t* pixels, const uint16_t * depth );


/***************************************************************************//**
@brief
   Sequence Reader for IMU and camera data.
*******************************************************************************/
typedef struct mvSRW_Reader mvSRW_Reader;



/***************************************************************************//**
@brief
   Initialize SequenceReader object.

@param configDir
   Location on storage where to read the sequence files.

@return
   Pointer to mvSRW_Reader object; returns NULL if failed.

@ingroup mvsrw
*******************************************************************************/
MV_API mvSRW_Reader* mvSRW_Reader_Initialize( const char* configDir );



/***************************************************************************//**
@brief
   De-initialize SequenceReader object.

@param pObj
   Pointer to SequenceReader object.

@ingroup mvsrw
*******************************************************************************/
void MV_API mvSRW_Reader_Deinitialize( mvSRW_Reader* pObj );



/***************************************************************************//**
@brief
   Get Number of Camera that the Reader found in Configuration (can be stereo
   and mono).

@param pObj
   Pointer to SequenceReader object.

@return
   Number of cameras.

@ingroup mvsrw
*******************************************************************************/
int MV_API mvSRW_Reader_GetNumberOfCameras( mvSRW_Reader* pObj );



/***************************************************************************//**
@brief
   Get the descriptors of the camera.

@param pObj
   Pointer to SequenceReader object.

@param cameras
   Preallocated memory for camera descriptors of available cameras.

@ingroup mvsrw
*******************************************************************************/
void MV_API mvSRW_Reader_GetCameras( mvSRW_Reader* pObj,
                                     mvCameraDescriptor* cameras );



/***************************************************************************//**
@brief
   Read camera parameters from file for camera with corresponding name.

@param pObj
   Pointer to SequenceReader object.

@param name
   Name of camera to use a id.

@param camera
   Preallocated memory for camera Configuration values.

@ingroup mvsrw
*******************************************************************************/
bool MV_API mvSRW_Reader_GetCameraParameters( mvSRW_Reader* pObj,
                                              const char* name,
                                              mvCameraConfiguration* camera );



/***************************************************************************//**
@brief
   Reads and returns the next frame (image + time)
   [1 image for monocular and 2 images for stereo].

@param pObj
   Pointer to SequenceReader object.

@return
   Newly allocated frame object that must be released after use.

@ingroup mvsrw
*******************************************************************************/
MV_API mvFrame* mvSRW_Reader_GetNextFrame( mvSRW_Reader* pObj );



/***************************************************************************//**
@brief
   skip over the next frame (not read image + time)
   [1 image for monocular and 2 images for stereo].

@param pObj
   Pointer to SequenceReader object.

@return
   Succeed or not

@ingroup mvsrw
*******************************************************************************/
MV_API bool mvSRW_Reader_SkipNextFrame( mvSRW_Reader* pObj );



/***************************************************************************//**
@brief
   Release frame data memory after use.

@param pObj
   Pointer to SequenceReader object.

@ingroup mvsrw
*******************************************************************************/
void MV_API mvSRW_Reader_ReleaseFrame( mvSRW_Reader* pObj, mvFrame* frame );



/***************************************************************************//**
@brief
   Returns the next gyro reading.

@param pObj
   Pointer to SequenceReader object.

@param maxTimestamp
   Read gyro readings up to but not exceeding given time stamp.

@return
   IMU data object that must be released after use.

@ingroup mvsrw
*******************************************************************************/
MV_API mvIMUData* mvSRW_Reader_GetNextGyro( mvSRW_Reader* pObj,
                                            int64_t maxTimestamp );


   /************************************************************************//**
   @detailed
      Returns the next accelerometer reading.
   @param pObj
      Pointer to SequenceReader object.
   @param maxTimestamp
      Read accelerometer readings up to but not exceeding given timestamp.
   @return
      IMU data object that must be released after use.
   ****************************************************************************/
   MV_API mvIMUData* mvSRW_Reader_GetNextAccel( mvSRW_Reader* pObj,
                                                int64_t maxTimestamp );


   /************************************************************************//**
   @detailed
      Release IMU data memory after use.
   @param pObj
      Pointer to SequenceReader object.
   ****************************************************************************/
   void MV_API mvSRW_Reader_ReleaseIMUData( mvSRW_Reader* pObj,
                                            mvIMUData* imu );


   /************************************************************************//**
   @detailed
      Returns the next gyro reading.
   @param obj
      Pointer to SequenceReader object.
   @param maxTimestamp
      Read GPS time sync readings up to but not exceeding given timestamp.
   @return
      GPS time sync data object that must be released after use.
   ****************************************************************************/
   MV_API mvGPStimeSyncData* mvSRW_Reader_GetNextGPStimeSync( mvSRW_Reader* obj,
                                                          int64_t maxTimestamp);


   /************************************************************************//**
   @detailed
      Release GPS time sync data memory after use.
   @param pObj
      Pointer to SequenceReader object.
   ****************************************************************************/
   void MV_API mvSRW_Reader_ReleaseGPStimeSyncData( mvSRW_Reader* pObj,
                                               mvGPStimeSyncData* timeSyncData);


   /************************************************************************//**
   @detailed
      Returns the next gyro reading.
   @param pObj
      Pointer to SequenceReader object.
   @param maxTimestamp
      Read GPS time sync readings up to but not exceeding given timestamp.
   @return
      GPS time sync data object that must be released after use.
   ****************************************************************************/
   MV_API mvGPSvelocityData* mvSRW_Reader_GetNextGPSvelocity( mvSRW_Reader* pObj,
                                                          int64_t maxTimestamp);


   /************************************************************************//**
   @detailed
      Release GPS velocity data memory after use.
   @param pObj
      Pointer to SequenceReader object.
   ****************************************************************************/
   void MV_API mvSRW_Reader_ReleaseGPSvelocityData( mvSRW_Reader* pObj,
                                               mvGPSvelocityData* velocityData);


   /************************************************************************//**
   @detailed
      Returns the next attitude reading.
   @param pObj
      Pointer to SequenceReader object.
   @param maxTimestamp
      Read attitude readings up to but not exceeding given timestamp.
   @return
      Attitude data object that must be released after use.
   ****************************************************************************/
   MV_API mvAttitudeData* mvSRW_Reader_GetNextAttitude( mvSRW_Reader* pObj,
                                                        int64_t maxTimestamp );


   /************************************************************************//**
   @detailed
      Release IMU data memory after use.
   @param pObj
      Pointer to SequenceReader object.
   ****************************************************************************/
   void MV_API mvSRW_Reader_ReleaseAttitudeData( mvSRW_Reader* pObj,
                                                 mvAttitudeData* attitude );

    /************************************************************************//**
   @detailed
      Release IMU data memory after use.
   @param pObj
      Pointer to SequenceReader object.
   ****************************************************************************/
   void MV_API mvSRW_Reader_ReleaseAttitudeData( mvSRW_Reader* pObj,
                                                 mvAttitudeData* attitude );


  /************************************************************************//**
   @detailed
      Reads MV standard XML Stereo Calibration file.
   @param filename
      Path to the calibration xml file.
   @return
      Pointer to mvStereoConfiguration object.  Caller is responsible for
      deallocation using delete if XML file is ill formed the function returns
      null.
   ****************************************************************************/
   MV_API bool mvSRW_Reader_GetStereoParameters( mvSRW_Reader* pObj,
                                                 const char* name,
                                                 mvStereoConfiguration* camera );

    /************************************************************************//**
   @detailed
      Writes Stereo configuration into MV standard XML format.
   @param filename
      Path to filename.
   @param stereoConfig
      Stereo configuration to writer.
   @return
      true on success false otherwise.
   ****************************************************************************/
   void MV_API mvSRW_Writer_AddStereoParameters( mvSRW_Writer* pObj, 
                                                 const char* filename,
                                                 mvStereoConfiguration* stereoConfig );


  /************************************************************************//**
   @detailed
      Writes camera extrinsic parameters to XML file.
   @param filename
      Path to the xml file.
   @return
      Pointer to mvCameraExtrinsicParameters object.
   ****************************************************************************/
   bool MV_API mvSRW_WriteCameraExtrinsicParameters( const char* filename,
                                    const mvCameraExtrinsicParameters* params );



/************************************************************************//**
@brief
   Reads camera extrinsic parameters from XML file.

@param filename
   Path to the XML file.

@param params
   Pointer to mvCameraExtrinsicParameters object.

@return
   true on success false otherwise.

@ingroup mvsrw
****************************************************************************/
bool MV_API mvSRW_ReadCameraExtrinsicParameters( const char* filename,
                                       mvCameraExtrinsicParameters* params );


#ifdef __cplusplus
}
#endif

#if defined _WIN32 && !defined MV_EXPORTS
#include "win/mvSRW_DLLGlue.h"
#endif


#endif
