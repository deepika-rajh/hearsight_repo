/*****************************************************************************
@copyright
Copyright (c) 2021 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/


#ifndef _RVDFS_H_
#define _RVDFS_H_

/***************************************************************************//**
@file
   rvDFS.h

@detailed
   Robot Vision,
   Depth From Stereo (DFS)

*******************************************************************************/


//==============================================================================
// Includes
//==============================================================================

#include <rv.h>

//==============================================================================
// Declarations
//==============================================================================

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct DFSBase rvDFS;

    typedef enum
    {
        RV_DFS_CVP = 0,   //CVP hardware mode
        RV_DFS_BOX,         //software solution, Box filter
        RV_DFS_GPU,
        RV_DFS_BILATERAL,    //software solution, bilateral filter
		RV_DFS_FASTGUIDED,    //fast guided filter
		RV_DFS_DOWNSAMPLE     //down sample the input images and up sample the output  
    }rvDFSMode;


    //------------------------------------------------------------------------------
    /// @param config_file
    ///   Path to config xml file
    /// @return
    ///   Pointer to dfs object; returns NULL if failed
    //------------------------------------------------------------------------------
    RV_API rvDFS* rvDFS_Initialize(rvDFSMode dfs_mode, int rows, int cols, const char* config_file);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Run rvDFS
    //------------------------------------------------------------------------------
    bool RV_API rvDFS_Execute(rvDFS* pHandle, int minDisparity, int levelDisparity, uint8_t* imgL, uint8_t* imgR, float* disparities);

    //------------------------------------------------------------------------------
    /// @detailed
    ///     Deinitialize DFS object.
    //------------------------------------------------------------------------------
    void RV_API rvDFS_Deinitialize(rvDFS* pHandle);


 
#ifdef __cplusplus
}
#endif


#endif
