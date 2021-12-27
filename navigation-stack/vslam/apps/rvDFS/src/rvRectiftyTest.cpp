/*****************************************************************************
@copyright
Copyright (c) 2021 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/


#include <string.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "dfs_factory.h"
#include "rvDFS.h"
#include "rvLog.h"
#include "dfs_test_tools.h"
#ifdef __linux__
#include <getopt.h>
#endif

#ifdef PROFILING
#include <chrono>
#endif

#ifdef TEST_RV_RECTIFY
#include "rv_opencv_rectify.h"
#include "rv_opencv_remap.h"
#endif

//#define DFS_CPP_STYLE_INTERFACE

using namespace std;
int RV_LOG_LEVEL = 1;
bool RV_STDERR_LOGGING = true;

// global variables
int gNumRows = 720, gNumCols = 1280;

rvDFSMode gRunningMode = rvDFSMode::RV_DFS_GPU;

string gLeftImage = "";
string gRightImage = "";
string gRectifyLeftImage = "";
string gRectifyRightImage = "";
string gStereoConfigFile = "";

void printHelp()
{
#ifdef __linux__
#else
	printf(
		"!!! On windows, rectified image asummed,\n Usage: leftImageName "
		"rightImageName imageWidth imageHeight\n imageWidth & imageHeight valid "
		"for yuv format only\n");
#endif
}

void parseCommandLine(int argc, char* argv[])
{
#ifdef __linux__
	int c, mode, loops;
	while ((c = getopt(argc, argv, "r:l:i:f:w:h:c:H")) != -1) {
		switch (c) {
		case 'l':
			gLeftImage = optarg;
			break;
		case 'r':
			gRightImage = optarg;
			break;
		case 'h':
			gNumRows = atoi(optarg);
			printf("gNumRows %d\n", gNumRows);
			break;
		case 'w':
			gNumCols = atoi(optarg);
			printf("gNumCols %d\n", gNumCols);
			break;
		case 'c':
			gStereoConfigFile = optarg;
			break;
		case 'H':
			printHelp();
			exit(1);
		case '?':
			if (optopt == 'm')
				printf("option -%c requires an argument\n", optopt);
			printHelp();
		default:
			printf("unknown argument\n");
			printHelp();
			exit(1);
		}
	}
#else
	gLeftImage = argv[1];
	gRightImage = argv[2];
	if (argc >= 5) {
		gRectifyLeftImage = argv[3];
		gRectifyRightImage = argv[4];
	}
	if (argc >= 6) {
		gStereoConfigFile = argv[5];
	}

#define MATPRINT(x) std::cout <<#x<<": "<< x << std::endl;
	MATPRINT(gLeftImage);
	MATPRINT(gRightImage);
	MATPRINT(gRectifyLeftImage);
	MATPRINT(gRectifyRightImage);
	MATPRINT(gStereoConfigFile);
#undef MATPRINT

#endif
	RV_DBG("leave parse Command Line");
}

void readImage(const char* imageName, cv::Mat& image)
{
	string name(imageName);
	if (name.substr(name.length() - 3) == "yuv")
	{
		image = cv::Mat(gNumRows, gNumCols, CV_8UC1);
		// Open left and right images
		FILE* inputFileL = fopen(name.c_str(), "rb");
		// Read left and right images
		fread((void*)image.data, sizeof(char), gNumCols * gNumRows, inputFileL);
		fclose(inputFileL);
	}
	else
	{
		image = cv::imread(imageName);
	}
	if (image.channels() == 3)
		cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
	gNumCols = image.cols;
	gNumRows = image.rows;
}


void testRectifyImage(const std::string& image_left_path, const std::string& image_right_path,
	const std::string& image_left_rectify, const std::string& image_right_rectify,
	const std::string& stereo_parameter_file)
{
	//load parameter files
	cv::FileStorage fs(stereo_parameter_file, cv::FileStorage::READ);
	if (!fs.isOpened()) return;

	cv::Mat camera_mat_left;
	fs["Camera_Matrix1"] >> camera_mat_left;
	cv::Mat distortion_left;
	fs["Distortion_Coefficients1"] >> distortion_left;
	cv::Mat camera_mat_right;
	fs["Camera_Matrix2"] >> camera_mat_right;
	cv::Mat distortion_right;
	fs["Distortion_Coefficients2"] >> distortion_right;
	cv::FileNode image_size_node = fs["Image_Size"];
	std::vector<int> image_size;
	image_size_node >> image_size;
	cv::Mat R;
	fs["R"] >> R;
	cv::Mat T;
	fs["T"] >> T;
	////////////
	//R = R.inv();
	//T = -R * T;
	/////////////



	cv::Mat leftImage, rightImage;
	readImage(image_left_path.c_str(), leftImage);
	readImage(image_right_path.c_str(), rightImage);
	cv::Mat ocvRotationL, ocvRotationR, ocvProjectionL, ocvProjectionR, Q;
	cv::Size cv_size(image_size[0], image_size[1]);
	cv::stereoRectify(camera_mat_left, distortion_left, camera_mat_right, distortion_right,
		cv_size, R, T, ocvRotationL, ocvRotationR, ocvProjectionL, ocvProjectionR, Q,
		cv::CALIB_ZERO_DISPARITY, 0);

#define MATPRINT(x) std::cout <<#x<<": "<< x << std::endl;
	MATPRINT(camera_mat_left);
	MATPRINT(distortion_left);
	MATPRINT(camera_mat_right);
	MATPRINT(distortion_right);
	MATPRINT(R);
	MATPRINT(T);
	MATPRINT(ocvRotationL);
	MATPRINT(ocvRotationR);
	MATPRINT(ocvProjectionL);
	MATPRINT(ocvProjectionR);
	MATPRINT(Q);
#undef MATPRINT

	cv::Mat mapL1, mapL2;
	cv::initUndistortRectifyMap(camera_mat_left,
		distortion_left,
		ocvRotationL,
		ocvProjectionL,
		cv_size,
		CV_32FC1,
		mapL1,
		mapL2);
	cv::Mat imageL_rect;
	cv::remap(leftImage,
		imageL_rect,
		mapL1,
		mapL2,
		cv::INTER_LINEAR);
	cv::imwrite(image_left_rectify, imageL_rect);

	cv::Mat mapR1, mapR2;
	cv::initUndistortRectifyMap(camera_mat_right,
		distortion_right,
		ocvRotationR,
		ocvProjectionR,
		cv_size,
		CV_32FC1,
		mapR1,
		mapR2);
	cv::Mat imageR_rect;
	cv::remap(rightImage,
		imageR_rect,
		mapR1,
		mapR2,
		cv::INTER_LINEAR);
	cv::imwrite(image_right_rectify, imageR_rect);
}

#ifdef TEST_RV_RECTIFY
void testRvRectifyImage(const std::string& image_left_path, const std::string& image_right_path,
	const std::string& image_left_rectify, const std::string& image_right_rectify,
	const std::string& stereo_parameter_file)
{
	rvStereoConfiguration stereo_parameter =
		dfs_test_tool::importStereoCalData(stereo_parameter_file);

	rv_rectify::OpencvRectification rv_rectify;
	rv_rectify.setParameter(stereo_parameter);
	double factor = rv_rectify.getDisparityToDepthFactor();
	float* map_xl = rv_rectify.getRemapXL();
	float* map_xr = rv_rectify.getRemapXR();
	float* map_yl = rv_rectify.getRemapYL();
	float* map_yr = rv_rectify.getRemapYR();


	rv_remap::OpencvRemap rv_remap_l(stereo_parameter.camera[0].pixelWidth,
		stereo_parameter.camera[0].pixelHeight, map_xl, map_yl);
	cv::Mat leftImage;
	readImage(image_left_path.c_str(), leftImage);
 	cv::Mat imageL_rect(stereo_parameter.camera[0].pixelHeight,
		stereo_parameter.camera[0].pixelWidth, CV_8U);
	rv_remap_l.remap(leftImage.ptr<uint8_t>(), imageL_rect.ptr<uint8_t>());
	cv::imwrite(image_left_rectify, imageL_rect);

	rv_remap::OpencvRemap rv_remap_r(stereo_parameter.camera[1].pixelWidth,
		stereo_parameter.camera[1].pixelHeight, map_xr, map_yr);
	cv::Mat rightImage;
	readImage(image_right_path.c_str(), rightImage);
	cv::Mat imageR_rect(stereo_parameter.camera[0].pixelHeight,
		stereo_parameter.camera[0].pixelWidth, CV_8U);
	rv_remap_r.remap(rightImage.ptr<uint8_t>(), imageR_rect.ptr<uint8_t>());
	cv::imwrite(image_right_rectify, imageR_rect);

}
#endif


int main(int argc, char* argv[])
{
	parseCommandLine(argc, argv);
	if (gStereoConfigFile.empty())
		return 0;

#ifdef TEST_RV_RECTIFY
	testRvRectifyImage(gLeftImage, gRightImage,
		gRectifyLeftImage, gRectifyRightImage,
		gStereoConfigFile);
#else
	testRectifyImage(gLeftImage, gRightImage,
		gRectifyLeftImage, gRectifyRightImage,
		gStereoConfigFile);
#endif
	return 0;
}

