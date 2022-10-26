/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef  __CAMARA_INTERFACE_H__
#define __CAMARA_INTERFACE_H__

#ifndef WIN32
#include <unistd.h>
#endif

#include <rvVWSLAM.h>

typedef void(*CameraCallback)(const int64_t, const unsigned char*, const unsigned short*);

class CameraInterface
{
public:
	void virtual addCallback(CameraCallback callBack) = 0;
	virtual const rvCameraParams& getCameraConfiguration() const = 0;
	virtual bool start() = 0;
	virtual bool stop() = 0;
};


#endif