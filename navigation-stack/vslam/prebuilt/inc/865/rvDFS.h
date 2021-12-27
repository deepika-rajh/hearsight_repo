/*****************************************************************************
* @copyright
* Copyright (c) 2021 Qualcomm Technologies, Inc.
* All Rights Reserved.
* Confidential and Proprietary - Qualcomm Technologies, Inc.
* *******************************************************************************/

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
		RV_DFS_CVP = 0,         //CVP hardware mode
		RV_DFS_BOX,             //Software solution, Box filter
		RV_DFS_GPU,             //OpenCL solution, box filter
		RV_DFS_BILATERAL,       //Software solution, bilateral filter
		RV_DFS_FASTGUIDED,      //Fast guided filter
		RV_DFS_GPU_GUIDED,      //OpenCL solution, guided filter
		RV_DFS_DOWNSAMPLE       //Down sample the input images and up sample the output
	}rvDFSMode;


	typedef struct
	{
		int minDisparity;                       //Minimum disparity level to search
		int numDisparityLevels;                 //Number of disparity levels
		int filterWidth;                        //Width of filter
		int filterHeight;                       //Height of filter
		bool doRectification=false;             //Indicate if rectification is needed before DFS
		bool doGpuRect;                         //Indicate if rectification on GPU
	}rvDFSParameter;


	//------------------------------------------------------------------------------
	/// @detailed
	///      Initialize rvDFS
	/// @param dfs_mode
	///   DFS mode
        /// @param width
        ///   Width of input images
        /// @param height
        ///   Height of input images
        /// @param stride
        ///   Stride of input images
        /// @param dfs_parameter
        ///   DFS parameters
        /// @param stereo_parameter
        ///   Intrinsic and extrinsic parameters of stereo camera
        /// @return
	///   Pointer to DFS object
	///   Returns NULL if failed
	//------------------------------------------------------------------------------
	RV_API rvDFS* rvDFS_Initialize(rvDFSMode dfs_mode, int width, int height, int stride,
		const rvDFSParameter dfs_parameter, const rvStereoConfiguration stereo_parameter);


	//------------------------------------------------------------------------------
	/// @detailed
	///     Get rectified camera parameters
	/// @param pHandle
        ///   Handle of rvDFS
	/// @return 
	///   return rvStereoConfiguration parameters
	//------------------------------------------------------------------------------
	rvStereoConfiguration RV_API rvDFS_GetRectifiedCameraParameter(rvDFS* pHandle);


	//------------------------------------------------------------------------------
	/// @detailed
	///     Run rvDFS and get disparity map
        /// @param pHandle
        ///   Handle of rvDFS
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
        /// @param disparity_map
        ///   Disparity map pointer for output
        /// @return
        ///   Returns True if success or False if failure
	//------------------------------------------------------------------------------
	bool RV_API rvDFS_CalculateDisparity(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, float* disparity_map);


        //------------------------------------------------------------------------------
        /// @detailed
        ///     Get rectified left and right images
        /// @param pHandle
        ///   Handle of rvDFS
        /// @param imgRectL
        ///   Left rectified image pointer
        /// @param imgRectR
        ///   Right rectified image pointer
        //------------------------------------------------------------------------------
        void RV_API rvDFS_GetRectImages(rvDFS* pHandle, uint8_t* imgRectL, uint8_t* imgRectR);


	//------------------------------------------------------------------------------
	/// @detailed
	///     Deinitialize DFS object
        /// @param pHandle
        ///   Handle of rvDFS
	//------------------------------------------------------------------------------
	void RV_API rvDFS_Deinitialize(rvDFS* pHandle);



#ifdef __cplusplus
}
#endif


#endif
