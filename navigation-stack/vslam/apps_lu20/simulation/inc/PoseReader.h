/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __POSE_READER_H__
#define __POSE_READER_H__

#include "rv.h"
#include <fstream>

class PoseReader
{
public:
   PoseReader(const std::string & wheelOdomName );
   ~PoseReader();

   bool getPose( uint64_t timestamp, rvPose6DRTWithTimestamp& pose );

private:
   bool getPose(rvPose6DRTWithTimestamp& pose );
   std::ifstream poseStream;
   rvPose6DRTWithTimestamp curPose;
};
#endif
