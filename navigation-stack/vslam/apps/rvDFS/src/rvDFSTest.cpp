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
#include <thread>
#endif

//#include <glog/logging.h>
#line 25 "rvDFSTest.cpp" 


//#define DFS_CPP_STYLE_INTERFACE

using namespace std;
int RV_LOG_LEVEL = 1;
bool RV_STDERR_LOGGING = true;

// global variables
int gNumRows = 480, gNumCols = 640, gStride = 640;
int gNumLoops = 1;
int gMinDisparity = 1;
int gLevelDisparity = 32;
rvDFSMode gRunningMode = rvDFSMode::RV_DFS_SPEED;
int gFPS = 0;
bool gDynamicRange = false;		// False: use disparity range defined in initialization, True: use disparity range set in disparity/depth/pointcloud APIs
int gOutputFormat = 0;			//0: disparity, 1: depth, 2: point cloud, 3: point cloud fusion with left image

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
		"-s"
		"\t arg denotes stride of input images\n"
		"-c"
		"\t arg denotes calibration parameter file\n"
		"-f"
		"\t arg denotes desired fps\n"
		"-o"
		"\t arg denotes output format 0: disparity, 1: depth, 2: point cloud, 3: point cloud fusion with left image"
		"-R"
		"\t arg sets dynamic disparity range, e.g. alternately use the first and second half of the range"
		"-H"
		"\t Print help information.\n";
		printf("%s\n", gHelp);
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
	bool strideNotSet=true;
	while ((c = getopt(argc, argv, "r:n:m:D:d:l:f:w:h:s:c:o:HR")) != -1) {
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
		case 'f':
			gFPS = atoi(optarg);
			break;
		case 'r':
			gRightImage = optarg;
			break;
		case 'h':
			gNumRows = atoi(optarg);
			break;
		case 'w':
			gNumCols = atoi(optarg);
			if (strideNotSet)
			{
				gStride = gNumCols;
			}
			break;
		case 's':
			gStride = atoi(optarg);
			strideNotSet = false;
			break;
		case 'c':
			gStereoConfigFile = optarg;
			break;
        case 'o':
			gOutputFormat = atoi(optarg);
			if(gOutputFormat > 3)
				gOutputFormat = 0;
			break;
		case 'R':
			gDynamicRange = true;
			break;
		case 'H':
			printHelp();
			exit(1);
		case '?':
			if (optopt == 'm')
				printf("option -%c requires an argument\n", optopt);
			printHelp();
			exit(1);
		default:
			printf("unknown argument\n");
			printHelp();
			exit(1);
		}
	}
#else
	gLeftImage = argv[1];
	gRightImage = argv[2];
	if (argc >= 4)
	{
		gRunningMode = (rvDFSMode)atoi(argv[3]);
	}
	if (argc >= 5)
	{
		gLevelDisparity = stoi(argv[4]);
	}
	if (argc >= 6)
	{
		gDoRectification = stoi(argv[5]);
	}
	if (argc >= 7)
	{
		gStereoConfigFile = argv[6];
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
		image = cv::Mat(gNumRows, gStride, CV_8UC1);
		// Open left and right images
		FILE* inputFileL = fopen(name.c_str(), "rb");
		// Read left and right images
		fread((void*)image.data, sizeof(char), gStride * gNumRows, inputFileL);
		fclose(inputFileL);
		//gNumCols, gNumRows and gStride relies on user input
	}
	else
	{
		image = cv::imread(imageName);
		if (gRightImage.empty())
		{
			gNumCols = image.cols / 2;
		}
		else
		{
			gNumCols = image.cols;
		}
		gStride = image.step;
		gNumRows = image.rows;
	}
	if (image.channels() == 3)
	{
		if (gRunningMode == 5)
		{
			cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
			RV_DBG("convert to RGB");
		}
		else if (gRunningMode != 4)
		{
			cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
			RV_DBG("convert to gray");
		}
		if (gRightImage.empty())
		{
			gNumCols = image.cols / 2;
		}
		else
		{
			gNumCols = image.cols;
		}
		gStride = image.step;
		gNumRows = image.rows;
	}

	RV_DBG("image %s size is %d x %d, stride is %d", imageName, gNumCols, gNumRows, gStride);
}

void RunTestC(cv::Mat& leftImage, cv::Mat& rightImage, cv::Mat* disparityMap, const rvStereoConfiguration& stereo_parameter, std::string fullFolder)
{
	std::cout << "Run C-style interface." << std::endl;

	if (!disparityMap)
	{
		std::cout << "Input disparity_map is empty" << std::endl;
		return;
	}
    rvDFSParameter dfs_parameter;
    dfs_parameter.filterHeight = 9;
    dfs_parameter.filterWidth = 15;
    dfs_parameter.disparity.minDisparity = gMinDisparity;
    dfs_parameter.disparity.numDisparityLevels = gLevelDisparity;
    dfs_parameter.doRectification = gDoRectification;
    dfs_parameter.doGpuRect = false;
	//For dynamic disparity range settings
	rvDFSDisparity dRange1,dRange2;

	rvDFSMode dfs_mode = rvDFSMode::RV_DFS_SPEED;
	if (gRunningMode == 0)
	{
		dfs_mode = rvDFSMode::RV_DFS_CVP;
	}
	else if (gRunningMode == 1)
	{
		dfs_mode = rvDFSMode::RV_DFS_COVERAGE;
	}
	else if (gRunningMode == 2)
	{
		dfs_mode = rvDFSMode::RV_DFS_SPEED;
		if (dfs_parameter.doRectification)
			dfs_parameter.doGpuRect = true;
	}
	else if (gRunningMode == 3)
	{
		dfs_mode = rvDFSMode::RV_DFS_ACCURACY;
	}

    if (gDynamicRange)
    {
        dRange1.minDisparity = gMinDisparity;
		dRange1.numDisparityLevels = gLevelDisparity/2;
		dRange2.minDisparity = gMinDisparity+dRange1.numDisparityLevels;
		dRange2.numDisparityLevels = gLevelDisparity - gLevelDisparity/2;
    }
	
	rvDFS* dfs_handle = rvDFS_Initialize(dfs_mode, gNumCols, gNumRows, gStride, dfs_parameter, stereo_parameter);
	if (dfs_handle == nullptr)
		return;
	rvStereoConfiguration rectified_stereo_parameter = rvDFS_GetRectifiedCameraParameter(dfs_handle);

	uint8_t* pLImg, * pRImg;
	pLImg = leftImage.ptr<uint8_t>();
	if (gRightImage.empty())
		pRImg = nullptr;
	else
		pRImg = rightImage.ptr<uint8_t>();
#ifdef PROFILING
	auto start = std::chrono::high_resolution_clock::now();
	if (gStereoConfigFile.empty())
	{
		for (int i = 0; i < gNumLoops; i++)
		{
			rvDFS_CalculateDisparity(dfs_handle, pLImg, pRImg, disparityMap->ptr<float>());
		}
	}
	else
	{
		for (int i = 0; i < gNumLoops; i++)
		{
			if(gOutputFormat==0)
			{
				if(!gDynamicRange)
				{
					rvDFS_CalculateDisparity(dfs_handle, pLImg, pRImg, disparityMap->ptr<float>());
				}
				else
				{
					if(i%2==0)
						rvDFS_CalculateDisparityWithNewDisparityRange(dfs_handle, pLImg, pRImg, disparityMap->ptr<float>(),&dRange1);
					else
						rvDFS_CalculateDisparityWithNewDisparityRange(dfs_handle, pLImg, pRImg, disparityMap->ptr<float>(),&dRange2);
				}
			}
			else if(gOutputFormat==1)
			{
				if(!gDynamicRange)
				{
					rvDFS_CalculateDepth(dfs_handle, pLImg, pRImg, disparityMap->ptr<float>());
				}
				else
				{
					if(i%2==0)
						rvDFS_CalculateDepthWithNewDisparityRange(dfs_handle, pLImg, pRImg, disparityMap->ptr<float>(),&dRange1);
					else
						rvDFS_CalculateDepthWithNewDisparityRange(dfs_handle, pLImg, pRImg, disparityMap->ptr<float>(),&dRange2);
				}
			}
			else 
			{
				RV_DBG("TBD, std::vector in C?");
			}
		}
	}
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	RV_ERR("Elapsed time: %f ms", elapsed.count() * 1000 / gNumLoops);
	if (!gStereoConfigFile.empty())
	{
		PointCloudType pcl;
		rvDFS_Depth2PointCloud(dfs_handle, disparityMap->ptr<float>(), &pcl);
		std::string ply_file = fullFolder + ("/point_cloud.ply");
		dfs_test_tool::writePLYPointcloud(ply_file, pcl, disparityMap->cols, disparityMap->rows);
	}
#else
	if (gStereoConfigFile.empty())
	{
		if (!rvDFS_CalculateDisparity(dfs_handle, pLImg, pRImg, disparityMap->ptr<float>()))
			return;
	}
	else
	{
		if (!rvDFS_CalculateDepth(dfs_handle, pLImg, pRImg, disparityMap->ptr<float>()))
			return;
		PointCloudType pcl;
		if (!rvDFS_Depth2PointCloud(dfs_handle, disparityMap->ptr<float>(), &pcl))
			return;
		std::string ply_file = fullFolder + ("/point_cloud.ply");
		dfs_test_tool::writePLYPointcloud(ply_file, pcl, disparityMap->cols, disparityMap->rows);
	}
#endif
	rvDFS_Deinitialize(dfs_handle);
}


bool RunTestCpp(cv::Mat& leftImage, cv::Mat& rightImage, cv::Mat* disparityMap, const rvStereoConfiguration& stereo_parameter, std::string fullFolder)
{
	if (!disparityMap)
	{
		return false;
	}

    rvDFSParameter dfs_parameter;
    dfs_parameter.filterHeight = 9;
    dfs_parameter.filterWidth = 15;
    dfs_parameter.disparity.minDisparity = gMinDisparity;
    dfs_parameter.disparity.numDisparityLevels = gLevelDisparity;
    dfs_parameter.doRectification = gDoRectification;
    dfs_parameter.doGpuRect = false;
	
	rvDFSDisparity dRange1,dRange2;		//For dynamic disparity range settings
	PointCloudType pcl;
	pcl.reserve(gNumCols*gNumRows*3);
	PointCloudColorType pclColor;
	pclColor.reserve(gNumCols*gNumRows*6);
	rvDFSMode dfs_mode = rvDFSMode::RV_DFS_SPEED;
	if (gRunningMode == 0)
	{
		dfs_mode = rvDFSMode::RV_DFS_CVP;
	}
	else if (gRunningMode == 1)
	{
		dfs_mode = rvDFSMode::RV_DFS_COVERAGE;
	}
	else if (gRunningMode == 2)
	{
		dfs_mode = rvDFSMode::RV_DFS_SPEED;
		if (gDoRectification)
			dfs_parameter.doGpuRect = true;
	}
	else if (gRunningMode == 3)
	{
		dfs_mode = rvDFSMode::RV_DFS_ACCURACY;
	}

	if (gDynamicRange)
    {
        dRange1.minDisparity = gMinDisparity;
		dRange1.numDisparityLevels = gLevelDisparity/2;
		dRange2.minDisparity = gMinDisparity+dRange1.numDisparityLevels;
		dRange2.numDisparityLevels = gLevelDisparity - gLevelDisparity/2;
    }
	
	std::shared_ptr<rv_dfs::DFSBase> dfs_base = rv_dfs::CreateDFSbase(dfs_mode);
	if (dfs_base == nullptr)
		return false;

	dfs_base->initialize(gNumCols, gNumRows, gStride, dfs_parameter, stereo_parameter);
	rvStereoConfiguration rectified_stereo_parameter = dfs_base->getRectifiedCameraParameter();
	uint8_t* pLImg, * pRImg;
	pLImg = leftImage.ptr<uint8_t>();
	if (gRightImage.empty())
		pRImg = nullptr;
	else
		pRImg = rightImage.ptr<uint8_t>();

	auto start = std::chrono::high_resolution_clock::now();
	if(gFPS>0)
	{
		auto durFrame = std::chrono::duration<double>(1.0/gFPS);
		auto thres = std::chrono::duration<double>(0.000001);
		auto tsPrev = start;
		for (int i = 0; i < gNumLoops; i++)
		{
			if(gStereoConfigFile.empty() || gOutputFormat == 0)
			{
				if(!gDynamicRange)
				{
					dfs_base->calculateDisparity(pLImg, pRImg, disparityMap->ptr<float>());
				}
				else
				{
					if(i%2==0)
						dfs_base->calculateDisparity(pLImg, pRImg, disparityMap->ptr<float>(),&dRange1);
					else
						dfs_base->calculateDisparity(pLImg, pRImg, disparityMap->ptr<float>(),&dRange2);
				}
			}
			else if(gOutputFormat==1)
			{
				if(!gDynamicRange)
				{
					dfs_base->calculateDepth(pLImg, pRImg, disparityMap->ptr<float>());
				}
				else
				{
					if(i%2==0)
						dfs_base->calculateDepth(pLImg, pRImg, disparityMap->ptr<float>(),&dRange1);
					else
						dfs_base->calculateDepth(pLImg, pRImg, disparityMap->ptr<float>(),&dRange2);
				}
			}
			else if(gOutputFormat==2)
			{
				if(!gDynamicRange)
				{
					dfs_base->calculatePointCloud(pLImg, pRImg, &pcl);
				}
				else
				{
					if(i%2==0)
						dfs_base->calculatePointCloud(pLImg, pRImg, &pcl,&dRange1);
					else
						dfs_base->calculatePointCloud(pLImg, pRImg, &pcl,&dRange2);
				}
			}
			else	//point cloud fusion with left image
			{
				if(!gDynamicRange)
				{
					dfs_base->calculatePointCloudColor(pLImg, pRImg, &pclColor);
				}
				else
				{
					if(i%2==0)
						dfs_base->calculatePointCloudColor(pLImg, pRImg, &pclColor,&dRange1);
					else
						dfs_base->calculatePointCloudColor(pLImg, pRImg, &pclColor,&dRange2);
				}
			}

			auto ts = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> dur = durFrame - (ts - tsPrev);
			// RV_ERR("dur %f, thres %f", dur.count(),thres.count());
			if(dur>thres)
			{
				
				std::this_thread::sleep_for(dur);
			}
			tsPrev = std::chrono::high_resolution_clock::now();
		}
	}
	else
	{
		for (int i = 0; i < gNumLoops; i++)
		{
			if(gStereoConfigFile.empty() || gOutputFormat == 0)
			{
				if(!gDynamicRange)
				{
					dfs_base->calculateDisparity(pLImg, pRImg, disparityMap->ptr<float>());
				}
				else
				{
					if(i%2==0)
						dfs_base->calculateDisparity(pLImg, pRImg, disparityMap->ptr<float>(),&dRange1);
					else
						dfs_base->calculateDisparity(pLImg, pRImg, disparityMap->ptr<float>(),&dRange2);
				}
			}
			else if(gOutputFormat==1)
			{
				if(!gDynamicRange)
				{
					dfs_base->calculateDepth(pLImg, pRImg, disparityMap->ptr<float>());
				}
				else
				{
					if(i%2==0)
						dfs_base->calculateDepth(pLImg, pRImg, disparityMap->ptr<float>(),&dRange1);
					else
						dfs_base->calculateDepth(pLImg, pRImg, disparityMap->ptr<float>(),&dRange2);
				}
			}
			else if(gOutputFormat==2)
			{
				if(!gDynamicRange)
				{
					dfs_base->calculatePointCloud(pLImg, pRImg, &pcl);
				}
				else
				{
					if(i%2==0)
						dfs_base->calculatePointCloud(pLImg, pRImg, &pcl,&dRange1);
					else
						dfs_base->calculatePointCloud(pLImg, pRImg, &pcl,&dRange2);
				}
			}
			else	//point cloud fusion with left image
			{
				if(!gDynamicRange)
				{
					dfs_base->calculatePointCloudColor(pLImg, pRImg, &pclColor);
				}
				else
				{
					if(i%2==0)
						dfs_base->calculatePointCloudColor(pLImg, pRImg, &pclColor,&dRange1);
					else
						dfs_base->calculatePointCloudColor(pLImg, pRImg, &pclColor,&dRange2);
				}
			}
		}
	}
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	RV_ERR("Elapsed time: %f ms", elapsed.count() * 1000 / gNumLoops);
	//LOG(INFO) << "Elapsed time (ms) is:   " << elapsed.count() * 1000 / gNumLoops;

	if(gStereoConfigFile.empty() || gOutputFormat == 0)		//save disparity map, original and false color images, as well as pfm format which is designed by middlebury
	{
		//save colorized disparity image
		cv::Mat disparityImageChar(disparityMap->size(), CV_8UC1);
		cv::Mat disparityImageFloat(disparityMap->size(), CV_32FC1);
		// dfs_base->getDisparity((float*)disparityImageFloat.data);
		unsigned char* pDisparityChar = (unsigned char*)disparityImageChar.data;
		float* pDisparityFloat = (float*)disparityMap->data;
		for (int ii = 0; ii < disparityMap->cols * disparityMap->rows; ++ii)
		{
			pDisparityChar[ii] = static_cast<unsigned char>(round(pDisparityFloat[ii]));
			if(gRunningMode == 2 && pDisparityFloat[ii] == 0.0)
				pDisparityFloat[ii] = INFINITY;		//gRunningMode 2 doesn't set INFINITY for disparity map
		}
		cv::imwrite(fullFolder + "/disparityOri.png", *disparityMap);
		double min;
		double max;
		cv::minMaxIdx(disparityImageChar, &min, &max);
		double scale = 255. / (max - min);
		disparityImageChar.convertTo(disparityImageChar, CV_8UC1, scale, -min * scale);
		cv::Mat falseColorsMap;
		cv::applyColorMap(disparityImageChar, falseColorsMap, cv::COLORMAP_JET);
		cv::imwrite(fullFolder + "/disparity.png", falseColorsMap);
/*
		//save "pfm" image for evaluation
		std::string pfmPath = fullFolder + "/disp0FCVF.pfm";
#ifdef WIN32
		WriteFilePFM((float*)disparityMap->data, disparityMap->cols, disparityMap->rows, pfmPath.c_str());
#else
		CShape sh(disparityMap->cols, disparityMap->rows, 1);
		CFloatImage fdisp;
		fdisp.ReAllocate(sh,(float*)disparityMap->data, false, sh.width*sizeof(float));
		WriteFilePFM(fdisp, pfmPath.c_str(), (float)(1.0/255.0));
#endif
*/
	}
	else if(gOutputFormat==1)		//depth map, save rectified images as well
	{
		cv::Mat rectLImg, rectRImg;
		if (gRunningMode == 5)
		{
			rectLImg = cv::Mat(gNumRows, gNumCols, CV_8UC3);
			rectRImg = cv::Mat(gNumRows, gNumCols, CV_8UC3);
		}
		else
		{
			rectLImg = cv::Mat(gNumRows, gNumCols, CV_8UC1);
			rectRImg = cv::Mat(gNumRows, gNumCols, CV_8UC1);
		}

		//save rectified images
		dfs_base->getRectImages(rectLImg.ptr<uint8_t>(), rectRImg.ptr<uint8_t>());
		cv::imwrite(fullFolder + "/rightRectifiedImage.png", rectRImg);
		cv::imwrite(fullFolder + "/leftRectifiedImage.png", rectLImg);
		
		//save raw depth image, assume the unit is centimeter
		cv::Mat depthImage(disparityMap->size(), CV_8UC1);
		unsigned char* pDepth = (unsigned char*)depthImage.data;
		float* pFloatDisparity = (float*)disparityMap->data;
		for (int ii = 0; ii < disparityMap->cols * disparityMap->rows; ++ii)
		{
			pDepth[ii] = static_cast<unsigned char>(round(pFloatDisparity[ii]*100.0));
		}
		cv::imwrite(fullFolder + "/depth.png", depthImage);

		//save colorized depth image
		double min;
		double max;
		cv::minMaxIdx(depthImage, &min, &max);
		double scale = 255. / (max - min);
		depthImage.convertTo(depthImage, CV_8UC1, scale, -min * scale);
		cv::Mat falseColorsMap;
		cv::applyColorMap(depthImage, falseColorsMap, cv::COLORMAP_JET);
		cv::imwrite(fullFolder + "/depthFalseColor.png", falseColorsMap);
	}
	else if(gOutputFormat==2)		//point cloud
	{
		//save cloud point
		std::string ply_file = fullFolder + ("/point_cloud.ply");
		dfs_test_tool::writePLYPointcloud(ply_file, pcl, disparityMap->cols, disparityMap->rows);
	}
	else	//point cloud color
	{
		//save cloud point fusion with gray scale left image
		std::string ply_file = fullFolder + ("/point_cloud_color.ply");
		dfs_test_tool::writePLYPointcloudColor(ply_file, pclColor, disparityMap->cols, disparityMap->rows);
	}

	return true;
}

void readRectifiedPara(rvStereoConfiguration& rectified_stereo_parameter, const std::string& file)
{
	//load parameter files
	cv::FileStorage fs(file, cv::FileStorage::READ);
	if (!fs.isOpened())
		return; //xsh
	cv::Mat P1;
	fs["P1"] >> P1;
	cv::Mat P2;
	fs["P2"] >> P2;
	//rectified_stereo_parameter
	double disparity_to_depth_factor_ = P2.at<double>(0, 3);
	double rectified_focal_length = P1.at<double>(0, 0);
	//update rectified_stereo_parameter_
	for (int k = 0; k < 2; ++k) {
		rectified_stereo_parameter.camera[k].focalLength[0] = rectified_focal_length;
		rectified_stereo_parameter.camera[k].focalLength[1] = rectified_focal_length;
		rectified_stereo_parameter.camera[k].principalPoint[0] = P1.at<double>(0, 2);
		rectified_stereo_parameter.camera[k].principalPoint[1] = P1.at<double>(1, 2);
		for (int i = 0; i < 8; ++i) {
			rectified_stereo_parameter.camera[k].distortion[i] = 0.0;
		}
	}
	for (int i = 0; i < 3; ++i)
	{
		rectified_stereo_parameter.rotation[i] = 0.0;
		rectified_stereo_parameter.translation[i] = 0.0;
	}
	if (rectified_focal_length != 0.0)
		rectified_stereo_parameter.translation[0] = disparity_to_depth_factor_ / rectified_focal_length;
	//xsh
}


void calDispWithSGBM(cv::Mat imgL, cv::Mat imgR, cv::Mat& imgDisparity8U)
{
	cv::Size imgSize = imgL.size();
	int numberOfDisparities = gLevelDisparity; // ((imgSize.width / 8) + 15) & -16;
	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(1, 16, 3);
	sgbm->setPreFilterCap(63);
	int SADWindowSize = 9;
	int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
	sgbm->setBlockSize(sgbmWinSize);

	int cn = imgL.channels();
	sgbm->setP1(8 * cn * sgbmWinSize * sgbmWinSize);
	sgbm->setP2(32 * cn * sgbmWinSize * sgbmWinSize);
	sgbm->setMinDisparity(1);
	sgbm->setNumDisparities(numberOfDisparities);
	sgbm->setUniquenessRatio(10);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setSpeckleRange(32);
	sgbm->setDisp12MaxDiff(1);

	int alg = cv::StereoSGBM::MODE_SGBM;
	if (alg == cv::StereoSGBM::MODE_HH)
		sgbm->setMode(cv::StereoSGBM::MODE_HH);
	else if (alg == cv::StereoSGBM::MODE_SGBM)
		sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
	else if (alg == cv::StereoSGBM::MODE_SGBM_3WAY)
		sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

	cv::Mat imgDisparity16S = cv::Mat(imgL.rows, imgL.cols, CV_16S);
#ifdef PROFILING
	auto start = std::chrono::high_resolution_clock::now();
	for (int i = 0; i < gNumLoops; i++)
	{
		sgbm->compute(imgL, imgR, imgDisparity16S);
	}
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;

	RV_ERR("SGBM Elapsed time: %f ms", elapsed.count() * 1000 / gNumLoops);
#else
	sgbm->compute(imgL, imgR, imgDisparity16S);
#endif
	cv::imwrite("disparitySGBM16S.bmp", imgDisparity16S);
	imgDisparity16S.convertTo(imgDisparity8U, CV_32F, 255 / (255 * 16.));
/*
#ifdef WIN32
	WriteFilePFM((float*)imgDisparity8U.data, imgDisparity8U.cols, imgDisparity8U.rows, "disp0SGBM32F.pfm");
#else
	CShape sh(imgDisparity8U.cols, imgDisparity8U.rows, 1);
	CFloatImage fdisp;
	fdisp.ReAllocate(sh, (float*)imgDisparity8U.data, false, sh.width * sizeof(float));
	WriteFilePFM(fdisp, "disp0SGBM32F.pfm", (float)(1.0 / 255.0));
#endif
*/
}

int main(int argc, char* argv[])
{
	parseCommandLine(argc, argv);

	//LOG_IF(FATAL, gLeftImage.empty()) << "Not specify the image path!";

	cv::Mat leftImage, rightImage;
	if (gRightImage.empty())
	{
		readImage(gLeftImage.c_str(), leftImage);
	}
	else
	{
		readImage(gLeftImage.c_str(), leftImage);
		readImage(gRightImage.c_str(), rightImage);
	}
	//LOG_IF(FATAL, gLeftImage.empty()) << "Error in reading " << gLeftImage;

	cv::Mat disp;
	disp = cv::Mat::zeros(gNumRows, gNumCols, CV_32FC1);

	// cv::Mat imgDisparity8U = cv::Mat(gNumRows, gNumCols, CV_8UC1);
//    calDispWithSGBM(leftImage, rightImage, imgDisparity8U);

	rvStereoConfiguration stereo_parameter;
	if (!gStereoConfigFile.empty())
	{
		//load parameter files
		stereo_parameter = dfs_test_tool::importStereoCalData(gStereoConfigFile);
	}

	std::string fullFolder = gLeftImage.c_str();
	int s = fullFolder.find_last_of("\\");
	if (s < 0)
		s = fullFolder.find_last_of("/");
	if (s <= 0)
		fullFolder = ".";
	else
		fullFolder.resize(s);


#ifdef DFS_CPP_STYLE_INTERFACE
	if (!RunTestCpp(leftImage, rightImage, &disp, stereo_parameter, fullFolder))
		std::cout << "Error in processing current image pair!";
#else
	RunTestC(leftImage, rightImage, &disp, stereo_parameter, fullFolder);
#endif

	//DLOG(INFO) << gLeftImage << " finished!";

	return 0;
}
