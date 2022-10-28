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
#include <array>

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
		RV_DFS_CVP = 0,         	            //CVP hardware mode
		RV_DFS_COVERAGE,                        //CPU solution, speed mode
		RV_DFS_SPEED,          	                //OpenCL solution, speed mode, fastest mode
		RV_DFS_ACCURACY,       	                //CPU solution, accuracy mode
	}rvDFSMode;

        typedef struct _rvDFSDisparity
        {
                int minDisparity;                       //Minimum disparity level to search
                int numDisparityLevels;                 //Number of disparity levels
        }rvDFSDisparity;

	typedef struct _rvDFSParameter
	{
		rvDFSDisparity  disparity;              //Number of disparity levels
		int filterWidth;                        //Width of filter
		int filterHeight;                       //Height of filter
		bool doRectification=false;             //Indicate if rectification is needed before DFS
		bool doGpuRect;                         //Indicate if rectification on GPU
	}rvDFSParameter;


        typedef std::vector<std::array<float,3>> PointCloudType;

        typedef std::vector<std::array<float,6>> PointCloudColorType;

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
	///     Run rvDFS and get disparity map with new disparity range
        /// @param pHandle
        ///   Handle of rvDFS
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
        /// @param disparityMap
        ///   Disparity map pointer for output
        /// @param dfs_disparity
        ///   DFS disparity parameters pointer, can be null
        /// @return
        ///   Returns True if success or False if failure
	//------------------------------------------------------------------------------
	bool RV_API rvDFS_CalculateDisparityWithNewDisparityRange(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, float* disparityMap, rvDFSDisparity* dfs_disparity);
	

	//------------------------------------------------------------------------------
	/// @detailed
	///     Run rvDFS and get depth map
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
	///     Run rvDFS and get depth map with new disparity range
        /// @param pHandle
        ///   Handle of rvDFS
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
	/// @param depthMap
	///   Depth map pointer for output
        /// @param dfs_disparity
        ///   DFS disparity parameters pointer, can be null
        /// @return
        ///   Returns True if success or False if failure
	//------------------------------------------------------------------------------
	bool RV_API rvDFS_CalculateDepthWithNewDisparityRange(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, float* depthMap, rvDFSDisparity* dfs_disparity);


        //------------------------------------------------------------------------------
	/// @detailed
	///     Run rvDFS and get point cloud
        /// @param pHandle
        ///   Handle of rvDFS
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
	/// @param pointCloud
        ///   Point cloud pointer for output
        /// @return
        ///   Returns True if success or False if failure
	//------------------------------------------------------------------------------
	bool RV_API rvDFS_CalculatePointCloud(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, PointCloudType* pointCloud);


        //------------------------------------------------------------------------------
	/// @detailed
	///     Run rvDFS and get point cloud with new disparity range
        /// @param pHandle
        ///   Handle of rvDFS
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
	/// @param pointCloud
        ///   Point cloud pointer for output
        /// @param dfs_disparity
        ///   DFS disparity parameters pointer, can be null
        /// @return
        ///   Returns True if success or False if failure
	//------------------------------------------------------------------------------
	bool RV_API rvDFS_CalculatePointCloudWithNewDisparityRange(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, PointCloudType* pointCloud, rvDFSDisparity* dfs_disparity);


        //------------------------------------------------------------------------------
	/// @detailed
	///     Run rvDFS and get point cloud fused with rectified left gray image
        /// @param pHandle
        ///   Handle of rvDFS
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
	/// @param pointCloud
        ///   Point cloud color pointer for output
        /// @return
        ///   Returns True if success or False if failure
	//------------------------------------------------------------------------------
	bool RV_API rvDFS_CalculatePointCloudColor(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, PointCloudColorType* pointCloud);


        //------------------------------------------------------------------------------
	/// @detailed
	///     Run rvDFS and get point cloud fused with rectified left gray image with new disparity range
        /// @param pHandle
        ///   Handle of rvDFS
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
	/// @param pointCloud
        ///   Point cloud color pointer for output
        /// @param dfs_disparity
        ///   DFS disparity parameters pointer, can be null
        /// @return
        ///   Returns True if success or False if failure
	//------------------------------------------------------------------------------
	bool RV_API rvDFS_CalculatePointCloudColorWithNewDisparityRange(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, PointCloudColorType* pointCloud, rvDFSDisparity* dfs_disparity);


        //------------------------------------------------------------------------------
        /// @detailed
        ///     Convert depth map to point cloud
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
        ///     Convert depth map to point cloud color
        /// @param pHandle
        ///   Handle of rvDFS
        /// @param imgRectL
        ///   rectified left image pointer
        /// @param depthMap
        ///   Depth map pointer
        /// @param pointCloud
        ///   Point cloud color pointer for output
        //------------------------------------------------------------------------------
        bool RV_API rvDFS_Depth2PointCloudColor(rvDFS* pHandle, uint8_t* imgRectL, float* depthMap, PointCloudColorType* pointCloud);


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
