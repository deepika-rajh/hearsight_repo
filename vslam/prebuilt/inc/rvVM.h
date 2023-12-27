/*****************************************************************************
@copyright
Copyright (c) 2019-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef RV_VM_H
#define RV_VM_H

/***************************************************************************//**
@file
   rvVM.h

@detailed
   Robot Vision,
   Navigation mapping
*******************************************************************************/


//==============================================================================
// Includes
//==============================================================================

#include "rv.h"
#include "mv.h"
#include "rvCamera.h"

#ifdef __cplusplus
extern "C"
{
#endif
    //==============================================================================
    /// @detailed
    ///     Navigation Mapping
    //==============================================================================
    typedef struct NAVMAP rvVM;


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Grip map state for PLANNER.
    //------------------------------------------------------------------------------
    typedef enum
    {
        RV_PLANNER_GRIDMAP_FREE = 0,
        RV_PLANNER_GRIDMAP_UNKNOWN = 255,
        RV_PLANNER_GRIDMAP_OCCUPIED = 254
    } RV_PLANNER_GRIDMAP_STATE;

    //------------------------------------------------------------------------------
    /// @detailed
    ///     Set map resolution from configuration file
    /// @param config_path
    ///     Pass path of configuration file
    /// @return
    ///     Return pointer of rvVM if succeeded, NULL if failed
    //------------------------------------------------------------------------------
    RV_API rvVM* rvVM_Initialize(const rvCameraIntrinsic* cameraConfig, const char* config_path);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Start the VM instance
    /// @param pObj
    ///     Pointer to VM object.
    //------------------------------------------------------------------------------
    RV_API void rvVM_Run(rvVM* pObj);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Sleep the VM instance
    /// @param pObj
    ///     Pointer to VM object.
    //------------------------------------------------------------------------------
    RV_API void rvVM_Sleep(rvVM* pObj);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Awake the VM instance
    /// @param pObj
    ///     Pointer to VM object.
    //------------------------------------------------------------------------------
    RV_API void rvVM_Awake(rvVM* pObj);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Stop the VM instance
    /// @param pObj
    ///     Pointer to VM object.
    //------------------------------------------------------------------------------
    RV_API void rvVM_Stop(rvVM* pObj);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Add new depth image to VM instance for mapping
    /// @param pObj
    ///     Pointer to VM object.
    /// @param imageData
    ///     Pointer to depth image.
    /// @param timestamp
    ///     Time stamp of the depth image.
    /// @return
    ///     True if succeeded, false if failed
    //------------------------------------------------------------------------------
    RV_API bool rvVM_AddOneImage(rvVM* pObj, const uint8_t* imageBuf, const unsigned short* depthData, const int64_t timestamp);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Add new pose to VM instance for mapping
    /// @param pObj
    ///     Pointer to VM object.
    /// @param pose
    ///     camera-in-world pose.
    /// @param timestamp
    ///     Time stamp of the depth image.
    /// @return
    ///     True if succeeded, false if failed
    //------------------------------------------------------------------------------
    RV_API bool rvVM_AddOnePose(rvVM* pObj, const rvPose6DRT pose, const int64_t timestamp);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     get grid map from VM.
    /// @param pObj
    ///     Pointer to VM object.
    /// @param gridImage
    ///     Pointer to grid map address. Please initialize to ensure "*gridImage == NULL".
    ///     If function returns false, "*gridImage" will be NULL.
    ///     If function returns true, Please release "*gridImage" soon after it is used.
    /// @param width
    ///     Pointer to grid map width. If return false, the width will be 0.
    /// @param height
    ///     Pointer to grid map height. If return false, the height will be 0.
    /// @param originX
    ///     Pointer to x origin of grid map.
    /// @param originY
    ///     Pointer to x origin grid map.
    /// @param timestamp
    ///     Pointer to timestamp when the grid map was lastly updated.
    /// @return
    ///     True if succeeded, false if failed
    //------------------------------------------------------------------------------
    RV_API bool rvVM_GetGridMap(rvVM* pObj, unsigned char** gridImage, int* width, int* height, int* originX, int* originY, int64_t* timestamp);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     get voxel map from VM
    /// @param pObj
    ///     Pointer to VM object.
    /// @param voxelPoints
    ///     Pointer to voxel memory. Please initialize to ensure "*voxelPoints == NULL".
    ///     If function returns 0, "*voxelPoints" will be NULL.
    ///     If function returns N>0, "*voxelPoints" will have data as "x1,y1,z1,x2,y2,z2,......,xN,yN,zN"
    ///     Please release "*voxelPoints" soon after it is used.
    /// @param timestamp
    ///     Pointer to timestamp when the voxel map was lastly updated.
    /// @return
    ///     Number of received voxel points
    //------------------------------------------------------------------------------
    RV_API int rvVM_GetVoxelMap(rvVM* pObj, float32_t** voxelPoints, int64_t* timestamp);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Clear the current map and restart to build a new map
    /// @param pObj
    ///     Pointer of rvVM object
    //------------------------------------------------------------------------------
    RV_API void rvVM_Restart(rvVM* pObj);

    //------------------------------------------------------------------------------
    /// @detailed
    ///     Delete the object before stopping the program
    /// @param pObj
    ///     Pointer of Pointer of rvVM object
    //------------------------------------------------------------------------------
    RV_API void rvVM_Deinitialize(rvVM* pObj);


    //------------------------------------------------------------------------------
    /// @detail
    ///     Save map
    /// @param pObj
    ///     Pointer of rvVM object
    /// @return
    ///     True if succeeded, false if failed
    //------------------------------------------------------------------------------
    RV_API bool rvVM_SaveGridMap(rvVM* pObj);

    RV_API bool rvVM_GetDepthImage(rvVM* pObj, uint16_t * depthImage );

#ifdef __cplusplus
}
#endif

#endif
