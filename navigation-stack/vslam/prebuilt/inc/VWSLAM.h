/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __VWSLAM_H__
#define __VWSLAM_H__
#include "rvCamera.h"

class VWSLAM
{
public:
   using  ptr = std::shared_ptr<VWSLAM>;

   typedef enum
   {
      QUALITY_SCALEESTIMATION = -3,
      QUALITY_FAILED = -2,
      QUALITY_INITIALIZING = -1,
      QUALITY_GREAT = 0,
      QUALITY_GOOD = 1,
      QUALITY_OK = 2,
      QUALITY_BAD = 3,
   } PoseQuality;

   typedef struct
   {
      PoseQuality poseQuality;
      struct
      {
         struct
         {
            float x, y, z;
         } position;
         struct
         {
            float roll, pitch, yaw;
         } euler;
      } pose;

      int64_t timestamp;
   } VWSLAMPose;

   typedef struct
   {
      typedef enum
      {
         MATCHING_OK,                        ///< Matching succeeded
         MATCHING_FAILED                     ///< Matching failed
      } OBSERVATION_STATE;

      float x = 0.f; //In pixel
      float y = 0.f; //in pixel
      OBSERVATION_STATE s = OBSERVATION_STATE::MATCHING_OK;
   } TrackedObservation;



   typedef struct
   {
      float _Brightness = 0.f;
      int32_t _KeyframeNum = 0;
      int32_t _MatchedMapPointNum = 0;
      int32_t _MisMatchedMapPointNum = 0;
      TrackedObservation * observationBuf = NULL;
   } VWSLAMStatus;


   virtual bool init( const std::string & root, const std::string &output, const rvCameraParams & cameraConfig ) = 0;
   virtual void run() = 0;
   virtual void stop() = 0;

   virtual void sleep() = 0;

   virtual void awake() = 0;

   virtual void reset() = 0;

   virtual void addWheelOdom( float linearVelocity, float angualVelocity,
                                  const float location[3], const float direction[4], int64_t timestamp ) = 0;

   virtual void addImu( const float linearAcceleration[3], const float angularVelocity[3], int64_t timestamp ) = 0;

   virtual void addImage( const int64_t timeStamp, const uint8_t * imageBuf, const uint16_t * depthBuf ) = 0;

   virtual float getWallAngle() const = 0;

   virtual VWSLAMPose getVslamOutputPose() = 0;

   virtual VWSLAMPose getVSLAMRawPose() = 0;
   
   virtual bool getUndistortedImage( uint8_t * img, int imageWidth, int imageHeight ) = 0;

   virtual bool getVWSLAMStatus( VWSLAMStatus & status ) = 0;

   virtual void addHijack( bool isHijack, int64_t timestamp ) = 0;

   virtual void getGridImage( unsigned char ** data, int& w, int& h ) = 0;

   virtual ~VWSLAM()
   {}
};

RV_API VWSLAM::ptr & getVWSLAM();

#endif /* ifndef __VWSLAM_H__ */
