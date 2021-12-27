/*****************************************************
Copyright (c) 2021 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*********************************************************/ 



#include <string.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "dfs_factory.h"
#include "rvDFS.h"
#include "rvLog.h"
#include "dfs_test_tools.h"
#ifdef __LINUX__
#include <getopt.h>
#endif

#ifdef PROFILING
#include <chrono>
#endif

//#define DFS_CPP_STYLE_INTERFACE

using namespace std;
int RV_LOG_LEVEL = 2;
bool RV_STDERR_LOGGING = true;

// global variables
int gNumRows = 480, gNumCols = 640;
size_t gNumLoops = 1;
int32_t gMinDisparity = 1;
int32_t gLevelDisparity = 32;

rvDFSMode gRunningMode = rvDFSMode::RV_DFS_GPU;

string gLeftImage = "";
string gRightImage = "";
string gStereoConfigFile = "";

bool gDoRectification = true;

void printHelp()
{
#ifdef __LINUX__
	static const char* gHelp =
		"USAGE: rv_dfs_test [-m arg] [-l arg] [-d arg] [-D "
		"arg] [-t arg] -[r arg] -[i arg] -[c arg]\n"
		"-m"
		"\t arg denotes running mode 0-CVP, 1-SW, 2-GPU, 3-Guided\n"
		"-t"
		"\t arg denotes directory where text version of disparity and inverse "
		"disparity are stored, by default text output is disabled\n"
		"-n"
		"\t arg denotes number of profiling loops in non-sequence mode\n"
		"-d"
		"\t arg denotes lower limit of disparity search range\n"
		"-D"
		"\t arg denotes upper limit of disparity search range\n"
		"-r"
		"\t arg denotes path of right input image\n"
		"-l"
		"\t arg denotes path of left input image\n"
		"-w"
		"\t arg denotes width of input images\n"
		"-h"
		"\t arg denotes hight of input images\n"
		"-c"
		"\t arg denotes calibration parameter file\n"
		"-H"
		"\t Print help information.\n";
	//	printf("%s\n", gHelp);
#else
	printf(
		"!!! On windows, rectified image asummed,\n Usage: leftImageName "
		"rightImageName imageWidth imageHeight\n imageWidth & imageHeight valid "
		"for yuv format only\n");
#endif
}

void parseCommandLine(int argc, char* argv[])
{
#ifdef __LINUX__
	int c, mode, loops;
	while ((c = getopt(argc, argv, "r:n:m:D:d:l:i:f:w:h:c:H")) != -1) {
		switch (c) {
		case 'm':
			gRunningMode = (rvDFSMode)atoi(optarg);
			break;
		case 'n':
			gNumLoops = atoi(optarg);
			break;
		case 'd':
			gMinDisparity = atoi(optarg);
			break;
		case 'D':
			gLevelDisparity = atoi(optarg);
			break;
		case 'l':
			gLeftImage = optarg;
			break;
		case 'r':
			gRightImage = optarg;
			break;
		case 'h':
			gNumRows = atoi(optarg);
			break;
		case 'w':
			gNumCols = atoi(optarg);
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
	if (argc >= 4) {
		gRunningMode = (rvDFSMode)atoi(argv[3]);
	}
	if (argc >= 5) {
		gStereoConfigFile = argv[4];
	}
	if (argc >= 6) {
		gLevelDisparity = stoi(argv[5]);
	}
	if (argc >= 7) {
		gDoRectification = stoi(argv[6]);
	}
#endif
	if (gStereoConfigFile.empty())
		gDoRectification = false;

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
	{
        if (gRunningMode == 5)
        {
            cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
            RV_DBG("convert to RGB");
        }
        else
        {
            cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
            RV_DBG("convert to gray");
        }
    }
	gNumCols = image.cols;
	gNumRows = image.rows;
	RV_DBG("image %s size is %d x %d", imageName, gNumCols, gNumRows);
}

void RunTestC(cv::Mat& leftImage, cv::Mat& rightImage, cv::Mat* disparityMap,
	const rvStereoConfiguration& stereo_parameter, rvStereoConfiguration* rectified_stereo_parameter)
{
	std::cout << "Run C-style interface." << std::endl;

	if (!disparityMap)
	{
		std::cout << "Input disparity_map is empty" << std::endl;
		return;
	}
    rvDFSParameter dfs_parameter;
    dfs_parameter.filterHeight = 11;
    dfs_parameter.filterWidth = 11;
    dfs_parameter.minDisparity = gMinDisparity;
    dfs_parameter.numDisparityLevels = gLevelDisparity;
    dfs_parameter.doRectification = gDoRectification;
    dfs_parameter.doGpuRect = false;

    rvDFSMode dfs_mode = rvDFSMode::RV_DFS_GPU;
	if (gRunningMode == 0)
	{
		dfs_mode = rvDFSMode::RV_DFS_CVP;
	}
	else if (gRunningMode == 1)
	{
		dfs_mode = rvDFSMode::RV_DFS_BOX;
	}
	else if (gRunningMode == 2)
	{
		dfs_mode = rvDFSMode::RV_DFS_GPU;
		dfs_parameter.doGpuRect = true;
	}
	else if (gRunningMode == 3)
	{
		dfs_mode = rvDFSMode::RV_DFS_BILATERAL;
	}
    else if (gRunningMode == 4)
    {
        dfs_mode = rvDFSMode::RV_DFS_FASTGUIDED;
    }
    else if (gRunningMode == 5)
    {
        dfs_mode = rvDFSMode::RV_DFS_GPU_GUIDED;
    }
    else if (gRunningMode == 6)
    {
        dfs_mode = rvDFSMode::RV_DFS_DOWNSAMPLE;
    }

	rvDFS* dfs_handle = rvDFS_Initialize(dfs_mode, gNumCols, gNumRows, gNumCols,
		dfs_parameter, stereo_parameter);
	if (dfs_handle == nullptr) return;
	*rectified_stereo_parameter = rvDFS_GetRectifiedCameraParameter(dfs_handle);
#ifdef PROFILING
	auto start = std::chrono::high_resolution_clock::now();
	for (int i = 0; i < gNumLoops; i++)
	{
		rvDFS_CalculateDisparity(dfs_handle, leftImage.ptr<uint8_t>(),
			rightImage.ptr<uint8_t>(), disparityMap->ptr<float>());
	}
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	RV_ERR("Elapsed time: %f ms", elapsed.count() * 1000 / gNumLoops);
#else
	if (!rvDFS_CalculateDisparity(dfs_handle, leftImage.ptr<uint8_t>(),
		rightImage.ptr<uint8_t>(), disparityMap->ptr<float>()))
		return;
#endif
	rvDFS_Deinitialize(dfs_handle);
}


void RunTestCpp(cv::Mat& leftImage, cv::Mat& rightImage, cv::Mat* disparityMap,
	const rvStereoConfiguration& stereo_parameter, rvStereoConfiguration* rectified_stereo_parameter)
{
	if (rectified_stereo_parameter == nullptr) {
		std::cout << "rectified_stereo_parameter is null" << std::endl;
		return;
	}
	std::cout << "Run Cpp-style interface." << std::endl;

	if (!disparityMap)
	{
		std::cout << "Input disparity_map is empty" << std::endl;
		return;
	}

    rvDFSParameter dfs_parameter;
    dfs_parameter.filterHeight = 15;
    dfs_parameter.filterWidth = 15;
    dfs_parameter.minDisparity = gMinDisparity;
    dfs_parameter.numDisparityLevels = gLevelDisparity;
    dfs_parameter.doRectification = gDoRectification;
    dfs_parameter.doGpuRect = false;

	rvDFSMode dfs_mode = rvDFSMode::RV_DFS_GPU;
	if (gRunningMode == 0)
	{
		dfs_mode = rvDFSMode::RV_DFS_CVP;
	}
	else if (gRunningMode == 1)
	{
		dfs_mode = rvDFSMode::RV_DFS_BOX;
	}
	else if (gRunningMode == 2)
	{
		dfs_mode = rvDFSMode::RV_DFS_GPU;
		dfs_parameter.doGpuRect = true;
	}
	else if (gRunningMode == 3)
	{
		dfs_mode = rvDFSMode::RV_DFS_BILATERAL;
	}
	else if (gRunningMode == 4)
	{
		dfs_mode = rvDFSMode::RV_DFS_FASTGUIDED;
	}
    else if (gRunningMode == 5)
    {
        dfs_mode = rvDFSMode::RV_DFS_GPU_GUIDED;
//        dfs_parameter.doGpuRect = true;
    }
    else if (gRunningMode == 6)
	{
		dfs_mode = rvDFSMode::RV_DFS_DOWNSAMPLE;
	}

	std::shared_ptr<rv_dfs::DFSBase> dfs_base = rv_dfs::CreateDFSbase(dfs_mode);
	if (dfs_base == nullptr) return;

	dfs_base->initialize(gNumCols, gNumRows, gNumCols, dfs_parameter,
		stereo_parameter);
	*rectified_stereo_parameter = dfs_base->getRectifiedCameraParameter();

#ifdef PROFILING
	auto start = std::chrono::high_resolution_clock::now();
	for (int i = 0; i < gNumLoops; i++)
	{
		dfs_base->calculateDisparity(leftImage.ptr<uint8_t>(),
			rightImage.ptr<uint8_t>(), disparityMap->ptr<float>());
		RV_DBG("i %d", i);
	}
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;

	RV_ERR("Elapsed time: %f ms", elapsed.count() * 1000 / gNumLoops);
#else
	dfs_base->calculateDisparity(leftImage.ptr<uint8_t>(),
		rightImage.ptr<uint8_t>(), disparityMap->ptr<float>());
#endif
//    start = std::chrono::high_resolution_clock::now();
//    for (int i = 0; i < gNumLoops; i++)
//    {
//        cv::Mat dispFinalMatF(gNumRows, gNumCols, CV_32FC1, disparityMap->ptr<float>());
//        cv::Mat dispFinalMat;
//        dispFinalMatF.convertTo(dispFinalMat, CV_16SC1);
//        cv::Mat tempMat(gNumRows,gNumCols,CV_16SC1);
//        cv::filterSpeckles( dispFinalMat, 0, 200, 2, tempMat );
//        dispFinalMat.convertTo(dispFinalMatF,CV_32FC1);
//    }
//    finish = std::chrono::high_resolution_clock::now();
//    elapsed = finish - start;
//    RV_ERR("filter speckle Elapsed time: %f ms", elapsed.count() * 1000 / gNumLoops);

    if(gDoRectification)
    {
        cv::Mat rectLImg(gNumRows, gNumCols, CV_8UC1);
        cv::Mat rectRImg(gNumRows, gNumCols, CV_8UC1);
        dfs_base->getRectImages(rectLImg.ptr<uint8_t>(), rectRImg.ptr<uint8_t>());
        cv::imwrite("leftRectifiedImage.bmp", rectLImg);
        cv::imwrite("rightRectifiedImage.bmp", rectRImg);
    }
	return;
}

int main(int argc, char* argv[])
{
	parseCommandLine(argc, argv);
	cv::Mat leftImage, rightImage;
	readImage(gLeftImage.c_str(), leftImage);
	readImage(gRightImage.c_str(), rightImage);
	cv::Mat disp;
	disp = cv::Mat::zeros(leftImage.size(), CV_32F);

	cv::Mat imgDisparity8U = cv::Mat(leftImage.rows, leftImage.cols, CV_8UC1);

	rvStereoConfiguration stereo_parameter;
	if (!gStereoConfigFile.empty()) {
		//load parameter files
		stereo_parameter =
			dfs_test_tool::importStereoCalData(gStereoConfigFile);
	}

	rvStereoConfiguration rectified_stereo_parameter;
#ifdef DFS_CPP_STYLE_INTERFACE
	RunTestCpp(leftImage, rightImage, &disp, stereo_parameter, &rectified_stereo_parameter);
#else
	RunTestC(leftImage, rightImage, &disp, stereo_parameter, &rectified_stereo_parameter);
#endif


//    cv::imwrite("disparityOrig.pfm", disp);
	cv::Mat disparityMap(disp.size(), CV_8UC1);
	for (int i = 0; i < disp.rows; ++i)
	{
		for (int j = 0; j < disp.cols; ++j)
		{
			disparityMap.at<uint8_t>(i, j) = static_cast<uint8_t>(disp.at<float>(i, j));
		}
	}
//	cv::imwrite("disparityOrig.bmp", disparityMap);

	double min;
	double max;
	cv::minMaxIdx(disparityMap, &min, &max);
	double scale = 255. / (max - min);
	disparityMap.convertTo(disparityMap, CV_8UC1, scale, -min * scale);
	cv::Mat falseColorsMap;
	cv::applyColorMap(disparityMap, falseColorsMap, cv::COLORMAP_JET);
	cv::imwrite("disparity.bmp", falseColorsMap);
	if (!gStereoConfigFile.empty()) {
		std::string ply_file("point_cloud.ply");
		dfs_test_tool::exportPLYFile(disp.cols, disp.rows, disp.cols, disp.ptr<float>(),
			rectified_stereo_parameter, ply_file);
	}
	return 0;
}
