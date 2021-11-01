/*****************************************************************************
 * @copyright
 * Copyright (c) 2021 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 * *******************************************************************************/


#ifndef RV_DFS_BASE_H
#define RV_DFS_BASE_H

#include <string>
#include "rvDFS.h"

namespace rv_dfs
{
	class DFSBase
	{
	public:
		virtual ~DFSBase() {}

		virtual bool initialize(int width, int height, int stride,
			const rvDFSParameter& dfs_parameter, const rvStereoConfiguration& stereo_parameter) = 0;
		virtual bool initialize(int width, int height, int stride, const std::string& config_file,
			const rvStereoConfiguration& stereo_parameter) = 0;
		virtual void calculateDisparity(uint8_t* imgL, uint8_t* imgR, float* disparities) = 0;
		virtual void calculateDepth(uint8_t* imgL, uint8_t* imgR, float* depth) = 0;

	protected:
		int width;
		int height;
		int stride;
		rvStereoConfiguration stereo_parameter_;
	};

}  // namespace rv_dfs

#endif
