/*****************************************************************************
@copyright
Copyright (c) 2021 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef RV_DFS_BASE_H
#define RV_DFS_BASE_H

#include <string>
#include "rvDFS.h"

namespace rv_dfs
{
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
			const rvDFSParameter& dfs_parameter, const rvStereoConfiguration& stereo_parameter) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and get disparity map
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
        /// @param disparity_map
        ///   Disparity map pointer for output
        //------------------------------------------------------------------------------
	virtual void calculateDisparity(uint8_t* imgL, uint8_t* imgR, float* disparities) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and get depth map
        ///     Not supported yet
        //------------------------------------------------------------------------------
	virtual void calculateDepth(uint8_t* imgL, uint8_t* imgR, float* depth) = 0;

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
        ///   return rvStereoConfiguration parameters
        //-----------------------------------------------------------------------------
	virtual rvStereoConfiguration getRectifiedCameraParameter() = 0;

	};

}  // namespace rv_dfs

#endif
