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
#include <rvDFS.h>
namespace dfs_test_tool {
	void writePLYPointcloud(const std::string& ply_file_path, const PointCloudType& pointCloud, size_t width, size_t height);
	void writePLYPointcloudColor(const std::string& ply_file_path, const PointCloudType& pointCloud, size_t width, size_t height);
	rvStereoConfiguration importStereoCalData(const std::string& file);
}
#endif
