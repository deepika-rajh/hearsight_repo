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
		RV_DFS_CVP = 0,         	//CVP hardware mode
		RV_DFS_SPEED_CPU,           //CPU solution, speed mode
		RV_DFS_SPEED_GPU,          	//OpenCL solution, speed mode, fastest mode
		RV_DFS_ACCURACY_CPU,       	//CPU solution, accuracy mode
		RV_DFS_COVERAGE_CPU,      	//CPU solution, coverage mode
		RV_DFS_COVERAGE_GPU,      	//OpenCL solution, coverage mode
	}rvDFSMode;


	typedef struct _rvDFSParameter
	{
		int minDisparity;                       //Minimum disparity level to search
		int numDisparityLevels;                 //Number of disparity levels
		int filterWidth;                        //Width of filter
		int filterHeight;                       //Height of filter
		bool doRectification=false;             //Indicate if rectification is needed before DFS
		bool doGpuRect;                         //Indicate if rectification on GPU
	}rvDFSParameter;


    typedef std::vector<std::vector<float>> PointCloudType;

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
        /// @param disparityMap
        ///   Disparity map pointer for output
        /// @return
        ///   Returns True if success or False if failure
	//------------------------------------------------------------------------------
	bool RV_API rvDFS_CalculateDisparity(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, float* disparityMap);
	

	//------------------------------------------------------------------------------
	/// @detailed
	///     Run rvDFS and get disparity map
        /// @param pHandle
        ///   Handle of rvDFS
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
	/// @param depthMap
	///   Depth map pointer for output
        /// @return
        ///   Returns True if success or False if failure
	//------------------------------------------------------------------------------
	bool RV_API rvDFS_CalculateDepth(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, float* depthMap);


        //------------------------------------------------------------------------------
        /// @detailed
        ///     Get rectified left and right images
        /// @param pHandle
        ///   Handle of rvDFS
        /// @param depthMap
        ///   Depth map pointer
        /// @param pointCloud
        ///   Point cloud pointer for output
        //------------------------------------------------------------------------------
        bool RV_API rvDFS_Depth2PointCloud(rvDFS* pHandle, float* depthMap, PointCloudType* pointCloud);
	
	
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
