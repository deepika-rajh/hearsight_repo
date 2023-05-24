/*****************************************************************************
@copyright
Copyright (c) 2022-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#ifndef _RVDFS_DFSTEST_TOOLS_
#define _RVDFS_DFSTEST_TOOLS_

#include <string>
#include <opencv2/opencv.hpp>
#include <rv.h>
#include <rvDFS.h>
#include <rv_dfs_base.h>
#include <dfs_factory.h>
namespace dfs_test_tool {
	typedef struct _stereoImagePath
	{
		std::string leftImage;
		std::string rightImage;
		int			index;
	}stereoImagePath;
	void writePLYPointCloud(const std::string& ply_file_path, const PointCloudType& pointCloud, size_t width, size_t height);
	void writePLYPointCloudColor(const std::string& ply_file_path, const PointCloudColorType& pointCloud, size_t width, size_t height);
	rvStereoCamera importStereoCalData(const std::string& file);
	void writeIntrinsic(const rvStereoCamera& param, const float factorW, const float factorH, const std::string& file);
	void scaleIntrinsics(rvStereoCamera& param, const float factorW, const float factorH);
	void processFolder(std::string dirPath, int minDisp, int dispLevel, bool doRect, int mode, int outputFormat, rvStereoCamera stereo_parameter);
	void saveColorizedDisparity(const cv::Mat& disparityFloat, const std::string& fullPath);
	void saveDepthImage(const cv::Mat& depthFloat, const std::string& fullPath);
	void saveColorizedDepthImage(const cv::Mat& depthFloat, const std::string& fullPath);
	void saveMap(const std::string& fullFolder, const std::string& mapName, int nameIdx, float* disparityFloat, int width, int height, int mode);
	void readImage(const char* imageName, cv::Mat& image, int* pWidth, int* pHeight, int* pStride, bool SBS);
}
#endif
