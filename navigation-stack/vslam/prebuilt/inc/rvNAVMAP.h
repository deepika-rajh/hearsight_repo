/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef _MV_NAVMAP_HEARDER_
#define _MV_NAVMAP_HEARDER_
 
//#include "rvNAVMAPImpl.h"
//#include "../src/NAVMAP/inc/rvNAVMAPImpl.h"
#include "rv.h"

// initialize the object
RV_API rvNAVMAP* rvNAVMAP_Initialize( const char* root_path );

// reset the voxel
RV_API void rvNAVMAP_Deepreset( rvNAVMAP* pObj );

// delete the object before stopping the program
RV_API void rvNAVMAP_Deinitialize( rvNAVMAP* pObj );

// add one depth image to update voxel map
RV_API bool rvNAVMAP_AddOneImage( rvNAVMAP* pObj, const uint16_t * imageData, const mvPose6DRT pose, mvCameraConfiguration cameraParam );

// get 2D grid map
RV_API bool rvNAVMAP_GetGridmap( rvNAVMAP* pObj, unsigned char **gridImage, int * width, int * height );

// get 3D voxels of with state "occupied"
RV_API unsigned int rvNAVMAP_GetVoxels( rvNAVMAP* pObj, float ** voxels );

#endif
