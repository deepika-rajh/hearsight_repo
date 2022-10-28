/*****************************************************************************
@copyright
Copyright (c) 2021 Qualcomm Technologies, Inc.
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
	void writePLYPointcloud(const std::string& ply_file_path, const PointCloudType& pointCloud, size_t width, size_t height);
	void writePLYPointcloudColor(const std::string& ply_file_path, const PointCloudColorType& pointCloud, size_t width, size_t height);
	rvStereoConfiguration importStereoCalData(const std::string& file);
	void processFolder(std::string dirPath, int minDisp, int dispLevel, bool doRect, int mode, int outputFormat, rvStereoConfiguration stereo_parameter);
	void saveMap(std::string& fullFolder, std::string& mapName, int nameIdx, float* disparityFloat, int width, int height, int mode);
	void readImage(const char* imageName, cv::Mat& image, int* pWidth, int* pHeight, int* pStride, bool SBS);
	void calDispWithSGBM(cv::Mat imgL, cv::Mat imgR, cv::Mat& imgDisparity8U, int dispLevel, int iterNum);
}
#endif
