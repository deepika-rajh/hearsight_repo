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
int RV_LOG_LEVEL = 1;
bool RV_STDERR_LOGGING = true;

// global variables
int gNumRows = 480, gNumCols = 640, gStride=640;
int gNumLoops = 1;
int32_t gMinDisparity = 1;
int32_t gLevelDisparity = 32;
rvDFSMode gRunningMode = rvDFSMode::RV_DFS_SPEED_GPU;

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
			gStride = gNumCols;
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
        else if(gRunningMode != 4)
        {
            cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
            RV_DBG("convert to gray");
        }
    }
	if(gRightImage.empty())
	{
		gNumCols = image.cols/2;
		gStride = image.step;
	}
	else
	{
		gNumCols = image.cols;
		gStride = image.step;
	}
	gNumRows = image.rows;
	RV_DBG("image %s size is %d x %d", imageName, gNumCols, gNumRows);
}

void RunTestC(cv::Mat& leftImage, cv::Mat& rightImage, cv::Mat* disparityMap,
	const rvStereoConfiguration& stereo_parameter, rvStereoConfiguration* rectified_stereo_parameter, std::string fullFolder)
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
    dfs_parameter.minDisparity = gMinDisparity;
    dfs_parameter.numDisparityLevels = gLevelDisparity;
    dfs_parameter.doRectification = gDoRectification;
    dfs_parameter.doGpuRect = false;

    rvDFSMode dfs_mode = rvDFSMode::RV_DFS_SPEED_GPU;
	if (gRunningMode == 0)
	{
		dfs_mode = rvDFSMode::RV_DFS_CVP;
	}
	else if (gRunningMode == 1)
	{
		dfs_mode = rvDFSMode::RV_DFS_SPEED_CPU;
	}
	else if (gRunningMode == 2)
	{
		dfs_mode = rvDFSMode::RV_DFS_SPEED_GPU;
		if(dfs_parameter.doRectification)
			dfs_parameter.doGpuRect = true;
	}
	else if (gRunningMode == 3)
	{
		dfs_mode = rvDFSMode::RV_DFS_ACCURACY_CPU;
	}
    else if (gRunningMode == 4)
    {
        dfs_mode = rvDFSMode::RV_DFS_COVERAGE_CPU;
    }
    else if (gRunningMode == 5)
    {
        dfs_mode = rvDFSMode::RV_DFS_COVERAGE_GPU;
    }
	
	rvDFS* dfs_handle = rvDFS_Initialize(dfs_mode, gNumCols, gNumRows, gStride, dfs_parameter, stereo_parameter);
	if (dfs_handle == nullptr) return;
	*rectified_stereo_parameter = rvDFS_GetRectifiedCameraParameter(dfs_handle);

	uint8_t *pLImg, *pRImg;
	pLImg = leftImage.ptr<uint8_t>();
	if(gRightImage.empty())
		pRImg = nullptr;
	else
		pRImg = rightImage.ptr<uint8_t>();
#ifdef PROFILING
	auto start = std::chrono::high_resolution_clock::now();
	if(gStereoConfigFile.empty())
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
			rvDFS_CalculateDepth(dfs_handle, pLImg, pRImg, disparityMap->ptr<float>());
		}
	}
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	RV_ERR("Elapsed time: %f ms", elapsed.count() * 1000 / gNumLoops);
	if(!gStereoConfigFile.empty())
	{
		PointCloudType pcl;
		rvDFS_Depth2PointCloud(dfs_handle, disparityMap->ptr<float>(), &pcl);
		std::string ply_file = fullFolder + ("/point_cloud.ply");
		dfs_test_tool::writePLYPointcloud(ply_file, pcl, disparityMap->cols, disparityMap->rows);
	}
#else
	if(gStereoConfigFile.empty())
	{
		if (!rvDFS_CalculateDisparity(dfs_handle, pLImg, pRImg, disparityMap->ptr<float>()))
			return;
	}
	else
	{
		if (!rvDFS_CalculateDepth(dfs_handle, pLImg, pRImg, disparityMap->ptr<float>()))
			return;
		PointCloudType pcl;
		if(!rvDFS_Depth2PointCloud(dfs_handle, disparityMap->ptr<float>(), &pcl))
			return;
		std::string ply_file = fullFolder + ("/point_cloud.ply");
		dfs_test_tool::writePLYPointcloud(ply_file, pcl, disparityMap->cols, disparityMap->rows);
	}
#endif
	rvDFS_Deinitialize(dfs_handle);
}


void RunTestCpp(cv::Mat& leftImage, cv::Mat& rightImage, cv::Mat* disparityMap,
	const rvStereoConfiguration& stereo_parameter, rvStereoConfiguration* rectified_stereo_parameter, std::string fullFolder)
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

	rvDFSMode dfs_mode = rvDFSMode::RV_DFS_COVERAGE_GPU;
	if (gRunningMode == 0)
	{
		dfs_mode = rvDFSMode::RV_DFS_CVP;
	}
	else if (gRunningMode == 1)
	{
		dfs_mode = rvDFSMode::RV_DFS_SPEED_CPU;
	}
	else if (gRunningMode == 2)
	{
		dfs_mode = rvDFSMode::RV_DFS_SPEED_GPU;
		if(gDoRectification)
		    dfs_parameter.doGpuRect = true;
        }
	else if (gRunningMode == 3)
	{
		dfs_mode = rvDFSMode::RV_DFS_ACCURACY_CPU;
	}
	else if (gRunningMode == 4)
	{
		dfs_mode = rvDFSMode::RV_DFS_COVERAGE_CPU;
	}
	else if (gRunningMode == 5)
	{
		dfs_mode = rvDFSMode::RV_DFS_COVERAGE_GPU;
//        dfs_parameter.doGpuRect = true;
	}

	std::shared_ptr<rv_dfs::DFSBase> dfs_base = rv_dfs::CreateDFSbase(dfs_mode);
	if (dfs_base == nullptr)
		return;

	//test seperate side by side input image
	// cv::Mat leftI = leftImage(cv::Rect(0,0,leftImage.cols/2,leftImage.rows)).clone();
	// cv::Mat rightI = leftImage(cv::Rect(leftImage.cols/2,0,leftImage.cols/2,leftImage.rows)).clone();
	// cv::imwrite("rightImage.bmp", rightI);
	// cv::imwrite("leftImage.bmp", leftI);
	// gStride = gNumCols;
	
	dfs_base->initialize(gNumCols, gNumRows, gStride, dfs_parameter, stereo_parameter);
	*rectified_stereo_parameter = dfs_base->getRectifiedCameraParameter();
	uint8_t *pLImg, *pRImg;
	pLImg = leftImage.ptr<uint8_t>();
	if(gRightImage.empty())
		pRImg = nullptr;
	else
		pRImg = rightImage.ptr<uint8_t>();

// //test seperate side by side input image
// 	pLImg = leftI.ptr<uint8_t>();
// 	pRImg = rightI.ptr<uint8_t>();
// 	cv::Mat leftImage1(gNumRows, gNumCols, CV_8UC1, pLImg);
// 	cv::Mat rightImage1(gNumRows, gNumCols, CV_8UC1, pRImg);
// 	cv::imwrite("leftImageb4input.bmp", leftImage1);
// 	cv::imwrite("rightImageb4input.bmp", rightImage1);
				

#ifdef PROFILING
	auto start = std::chrono::high_resolution_clock::now();
	if(gStereoConfigFile.empty())
	{
		for (int i = 0; i < gNumLoops; i++)
		{
			dfs_base->calculateDisparity(pLImg, pRImg, disparityMap->ptr<float>());
		}
	}
	else
	{
		for (int i = 0; i < gNumLoops; i++)
		{
			dfs_base->calculateDepth(pLImg, pRImg, disparityMap->ptr<float>());
		}
	}
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	RV_ERR("Elapsed time: %f ms", elapsed.count() * 1000 / gNumLoops);
	if(!gStereoConfigFile.empty())
	{
		cv::Mat rectLImg, rectRImg;
		if(gRunningMode == 5)
		{
			rectLImg = cv::Mat(gNumRows, gNumCols, CV_8UC3);
        	rectRImg = cv::Mat(gNumRows, gNumCols, CV_8UC3);
		}
        else
		{
			rectLImg = cv::Mat(gNumRows, gNumCols, CV_8UC1);
        	rectRImg = cv::Mat(gNumRows, gNumCols, CV_8UC1);
		}
		// if(pRImg==nullptr)
		// 	dfs_base->getRectImages(rectLImg.ptr<uint8_t>(), nullptr);
		// else
		{
			dfs_base->getRectImages(rectLImg.ptr<uint8_t>(), rectRImg.ptr<uint8_t>());
			cv::imwrite("rightRectifiedImage.bmp", rectRImg);
		}
		cv::imwrite("leftRectifiedImage.bmp", rectLImg);
		
		PointCloudType pcl;
		dfs_base->depth2PointCloudColor(rectLImg.ptr<uint8_t>(), disparityMap->ptr<float>(), &pcl);
		std::string ply_file = fullFolder + ("/point_cloud.ply");
		dfs_test_tool::writePLYPointcloudColor(ply_file, pcl, disparityMap->cols, disparityMap->rows);
	}
#else
	if(gStereoConfigFile.empty())
	{
		dfs_base->calculateDisparity(pLImg, pRImg, disparityMap->ptr<float>());
	}
	else
	{
		if(!dfs_base->calculateDepth(pLImg, pRImg, disparityMap->ptr<float>()))
			return;
		PointCloudType pcl;
		cv::Mat rectLImg(gNumRows, gStride, CV_8UC1);
		cv::Mat rectRImg(gNumRows, gNumCols, CV_8UC1);
		if(!dfs_base->depth2PointCloudColor(rectLImg.ptr<uint8_t>(), disparityMap->ptr<float>(), &pcl))
			return;
		std::string ply_file = fullFolder + ("/point_cloud.ply");
		dfs_test_tool::writePLYPointcloudColor(ply_file, pcl, disparityMap->cols, disparityMap->rows);
	}
#endif

    // if(gDoRectification)
    // {
	// 	if(gRunningMode == 5)
	// 	{
	// 		cv::Mat rectLImg(gNumRows, gNumCols, CV_8UC3);
    //     	cv::Mat rectRImg(gNumRows, gNumCols, CV_8UC3);
	// 		dfs_base->getRectImages(rectLImg.ptr<uint8_t>(), rectRImg.ptr<uint8_t>());
	// 		cv::imwrite("leftRectifiedImage.bmp", rectLImg);
	// 		cv::imwrite("rightRectifiedImage.bmp", rectRImg);
	// 	}
    //     else
	// 	{
	// 		cv::Mat rectLImg(gNumRows, gStride, CV_8UC1);
    //     	cv::Mat rectRImg(gNumRows, gNumCols, CV_8UC1);
	// 		dfs_base->getRectImages(rectLImg.ptr<uint8_t>(), rectRImg.ptr<uint8_t>());
	// 		cv::imwrite("leftRectifiedImage.bmp", rectLImg);
	// 		cv::imwrite("rightRectifiedImage.bmp", rectRImg);
	// 	}
    // }
	return;
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


void calDispWithSGBM(cv::Mat imgL, cv::Mat imgR, cv::Mat &imgDisparity8U)
{
    cv::Size imgSize = imgL.size();
    int numberOfDisparities = gLevelDisparity; // ((imgSize.width / 8) + 15) & -16;
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(1, 16, 3);
    sgbm->setPreFilterCap(63);
    int SADWindowSize = 9;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);

    int cn = imgL.channels();
    sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
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
    imgDisparity16S.convertTo(imgDisparity8U, CV_32F , 255 / (255*16.));
#ifdef WIN32
	//WriteFilePFM((float*)imgDisparity8U.data, imgDisparity8U.cols, imgDisparity8U.rows, "disp0SGBM32F.pfm");
#else
	//CShape sh(imgDisparity8U.cols, imgDisparity8U.rows, 1);
	//CFloatImage fdisp;
	//fdisp.ReAllocate(sh,(float*)imgDisparity8U.data, false, sh.width*sizeof(float));
	//WriteFilePFM(fdisp, "disp0SGBM32F.pfm",(float)(1.0/255.0));
#endif
}

int main(int argc, char* argv[])
{
	parseCommandLine(argc, argv);
	cv::Mat leftImage, rightImage;
	if(gRightImage.empty())
	{
		gStride*=2;
		readImage(gLeftImage.c_str(), leftImage);
		if(gRunningMode!=2)
		{
			gStride /= 2;
		}
	}
	else
	{
		readImage(gLeftImage.c_str(), leftImage);
		readImage(gRightImage.c_str(), rightImage);
	}
	
	cv::Mat disp;
	disp = cv::Mat::zeros(gNumRows, gNumCols, CV_32F);

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

	rvStereoConfiguration rectified_stereo_parameter;
#ifdef DFS_CPP_STYLE_INTERFACE
	RunTestCpp(leftImage, rightImage, &disp, stereo_parameter, &rectified_stereo_parameter, fullFolder);
#else
	RunTestC(leftImage, rightImage, &disp, stereo_parameter, &rectified_stereo_parameter, fullFolder);
#endif

	if(gStereoConfigFile.empty()) // get disparity from DFS
	{
		cv::Mat disparityMap(disp.size(), CV_8UC1);
		for (int i = 0; i < disp.rows; ++i)
		{			
			for (int j = 0; j < disp.cols; ++j)
			{
				disparityMap.at<uint8_t>(i, j) = static_cast<uint8_t>(std::round(disp.at<float>(i, j)));
				if (disp.at<float>(i, j) == 0.0)
				{
					disp.at<float>(i, j) = INFINITY;
				}
			}
		}
		cv::imwrite(fullFolder + "/disparityOrig.bmp", disparityMap);

		double min;
		double max;
		cv::minMaxIdx(disparityMap, &min, &max);
		double scale = 255. / (max - min);
		disparityMap.convertTo(disparityMap, CV_8UC1, scale, -min * scale);
		cv::Mat falseColorsMap;
		cv::applyColorMap(disparityMap, falseColorsMap, cv::COLORMAP_JET);
		cv::imwrite(fullFolder + "/disparity.bmp", falseColorsMap);
		std::string pfmPath = fullFolder + "/disp0FCVF.pfm";
	#ifdef WIN32
		//WriteFilePFM((float*)disp.data, disp.cols, disp.rows, pfmPath.c_str());
	#else
		//CShape sh(disparityMap.cols, disparityMap.rows, 1);
		//CFloatImage fdisp;
		//fdisp.ReAllocate(sh,(float*)disp.data, false, sh.width*sizeof(float));
		//WriteFilePFM(fdisp, pfmPath.c_str(), (float)(1.0/255.0));
	#endif
	}
	else // get depth from DFS
	{
		cv::Mat depthImage(disp.size(), CV_16UC1);
		for (int i = 0; i < disp.rows; ++i)
		{
			for (int j = 0; j < disp.cols; ++j)
			{
				depthImage.at<unsigned short>(i, j) = static_cast<unsigned short>(disp.at<float>(i, j));
			}
		}
		cv::imwrite(fullFolder + "/depth.png", depthImage);

		double min;
		double max;
		cv::minMaxIdx(depthImage, &min, &max);
		double scale = 255. / (max - min);
		depthImage.convertTo(depthImage, CV_8UC1, scale, -min * scale);
		cv::Mat falseColorsMap;
		cv::applyColorMap(depthImage, falseColorsMap, cv::COLORMAP_JET);
		cv::imwrite(fullFolder + "/depthColor.png", falseColorsMap);
    }
	return 0;
}
