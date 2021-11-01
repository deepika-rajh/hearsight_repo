/*****************************************************************************
@copyright
Copyright (c) 2021 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/



#ifndef _RVDFS_DFSTEST_TOOLS_
#define _RVDFS_DFSTEST_TOOLS_

#include <string>

#include <rv.h>
namespace dfs_test_tool {
	void exportPLYFile(int width, int height, int stride,
		const float* disparity_map, const rvStereoConfiguration& stereo_config,
		const std::string& ply_file_path);

	rvStereoConfiguration importStereoCalData(const std::string& file);
}
#endif
