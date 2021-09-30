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
		virtual ~DFSBase() {}

		virtual bool initialize(int rows, int cols, const std::string& config_file) = 0;
		virtual void calculateDisparity(int minDisparity, int levelDisparity, uint8_t* imgL, uint8_t* imgR, float* disparities) = 0;

	protected:
		int height;
		int width;
	};

}  // namespace rv_dfs

#endif
