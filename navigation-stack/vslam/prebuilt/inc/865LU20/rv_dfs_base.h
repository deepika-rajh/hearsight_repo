/*****************************************************************************
@copyright
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/


#ifndef RV_DFS_BASE_H
#define RV_DFS_BASE_H

#include <string>
#include "rvDFS.h"

namespace rv_dfs
{
    /// flags for annotating disparity maps
    enum eDFSPixelState
    {
        DFS_PIXEL_ATTEMPTED = 0,
        DFS_PIXEL_CORRECTED = 1 << 1,           /// disparity corrected by post-processing
        DFS_PIXEL_FILLED = 1 << 2,              /// disparity filled by post-processing
        DFS_PIXEL_SMALLCONCOMP = 1 << 3,
        DFS_PIXEL_OK = 1 << 4,                  /// disparity at given pixel determined
        DFS_PIXEL_UNKNOWN = 1 << 5,             /// disparity at given pixel unknown
        DFS_PIXEL_OCCLUDED = 1 << 6,            /// pixel occluded        
        DFS_PIXEL_NOTTEXTURED = 1 << 7
    };

	class DFSBase
	{
	public:
        //------------------------------------------------------------------------------
        /// @detailed
        ///      Initialize
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
        ///   Returns True if success or False if failure
        //------------------------------------------------------------------------------
	    virtual bool initialize(int width, int height, int stride,
			                    const rvDFSParameter& dfs_parameter, const rvStereoCamera& stereo_parameter) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and get disparity map
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
        /// @param disparities
        ///   Disparity map pointer for output
        /// @param dfs_disparity
        ///   DFS disparity parameters pointer, can be null
        //------------------------------------------------------------------------------
	    virtual void calculateDisparity(uint8_t* imgL, uint8_t* imgR, float* disparities, rvDFSDisparity* dfs_disparity=nullptr) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and get depth map
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
        /// @param depth
        ///   Depth map pointer for output
        /// @param dfs_disparity
        ///   DFS disparity parameters pointer, can be null
        //------------------------------------------------------------------------------
	    virtual bool calculateDepth(uint8_t* imgL, uint8_t* imgR, float* depth, rvDFSDisparity* dfs_disparity=nullptr) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and get point cloud
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
        /// @param pointCloud
        ///   point cloud pointer to pointer for output
        /// @param dfs_disparity
        ///   DFS disparity parameters pointer, can be null
        //------------------------------------------------------------------------------
	    virtual bool calculatePointCloud(uint8_t* imgL, uint8_t* imgR, PointCloudType* pointCloud, rvDFSDisparity* dfs_disparity=nullptr) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and get point cloud with color image
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
        /// @param pointCloud
        ///   point cloud pointer to pointer for output
        /// @param dfs_disparity
        ///   DFS disparity parameters pointer, can be null
        //------------------------------------------------------------------------------
	    virtual bool calculatePointCloudColor(uint8_t* imgL, uint8_t* imgR, PointCloudColorType* pointCloud, rvDFSDisparity* dfs_disparity=nullptr) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and get disparity/depth/point cloud
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
        /// @param disparities
        ///   Disparity map pointer for output
        /// @param depth
        ///   Depth map pointer for output
        /// @param pointCloud
        ///   point cloud pointer to pointer for output
        /// @param dfs_disparity
        ///   DFS disparity parameters pointer, can be null
        //------------------------------------------------------------------------------
	    virtual bool calculateDispDepthPointCloud(uint8_t* imgL, uint8_t* imgR, float* disparities, float* depth, PointCloudType* pointCloud, rvDFSDisparity* dfs_disparity=nullptr) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and get disparity/depth/point cloud with color image
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
        /// @param disparities
        ///   Disparity map pointer for output
        /// @param depth
        ///   Depth map pointer for output
        /// @param pointCloud
        ///   Point cloud pointer for output
        /// @param pointCloudColor
        ///   Point cloud color pointer for output
        /// @param dfs_disparity
        ///   DFS disparity parameters pointer, can be null
        //------------------------------------------------------------------------------
	    virtual bool calculateDispDepthPointCloudColor(uint8_t* imgL, uint8_t* imgR, float* disparities, float* depth, PointCloudColorType* pointCloudColor, rvDFSDisparity* dfs_disparity=nullptr) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and convert depth map to point cloud
        /// @param depth
        ///   depth map pointer
        /// @param pointCloud
        ///   point cloud pointer to pointer for output
        //------------------------------------------------------------------------------
        virtual bool depth2PointCloud(float* depth, PointCloudType* pointCloud) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and convert depth map to point cloud with gray color
        /// @param imgRectL
        ///   rectified left image pointor
        /// @param depth
        ///   depth map pointer
        /// @param pointCloud
        ///   point cloud pointer to pointer for output
        //------------------------------------------------------------------------------
        virtual bool depth2PointCloudColor(uint8_t* imgRectL, float* depth, PointCloudColorType* pointCloud) = 0;


        //------------------------------------------------------------------------------
        /// @detailed
        ///     Get rectified left and right images
        /// @param imgRectL
        ///   Left rectified image pointer
        /// @param imgRectR
        ///   Right rectified image pointer
        //------------------------------------------------------------------------------
        virtual void getRectImages(uint8_t* imgRectL, uint8_t* imgRectR) = 0;

	    //------------------------------------------------------------------------------
        /// @detailed
        ///     Get rectified camera parameters
        /// @return
        ///   return rvStereoCamera parameters
        //-----------------------------------------------------------------------------
	    virtual rvStereoCamera getRectifiedCameraParameter() = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Get disparity map. Can be used after calculateDepth/calculatePointCloud/calculatePointCloudColor.
        ///     Only valid for RV_DFS_COVERAGE and RV_DFS_SPEED modes
        /// @param disparities
        ///   Disparity map pointer for output
        //------------------------------------------------------------------------------
        virtual void getDisparity(float* disparities) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     De-initialize 
        /// @param disparities
        ///   Disparity map pointer for output
        //------------------------------------------------------------------------------
        virtual void deInitialize() = 0;
	};

}  // namespace rv_dfs

#endif
