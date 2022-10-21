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
#ifdef WIN32
#include "pfm.h"
#elif MIDDLEBERRY_EVAL
#include "imageLib.h"		//MiddEval3 SDK for middlebury dataset
#endif
#include "dfs_factory.h"
#include "rvDFS.h"
#include "rvLog.h"
#include "dfs_test_tools.h"
#ifdef __LINUX__
#include <getopt.h>
#endif
#include <boost/filesystem.hpp>
#ifdef PROFILING
#include <chrono>
#include <thread>
#endif

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
int gOutputFormat = 2;			//0: disparity, 1: depth, 2: point cloud, 3: point cloud fusion with left image
string gLeftImage = "";
string gRightImage = "";
string gStereoConfigFile = "";
bool gDoRectification = false;	//False: input image pairs are undistorted and rectified, if stereo config file is provided, the parameters are rectified stereo parameters
								//True: input image pairs should be undistorted or rectified with config file, stereo config file must be provided

bool gVerbose = false;

/*----------------------------------------------------------------------------*/
#if defined(WIN32) || defined(_WIN32)
const char PathDelimiter = '\\';
#else
const char PathDelimiter = '/';
#endif // WIN32
/*----------------------------------------------------------------------------*/
/* print out usage help text */
void    printHelp(const char * argv0)
{
    string exename = argv0;
    string exepath;
    size_t pos = exename.rfind(PathDelimiter);

    if (pos==string::npos)
        exepath = PathDelimiter;
    else
        exepath = exename.substr(0, pos+1);
    exename.erase(0, pos+1);

    for(int i=0; i<49; i++) putchar('-');
    printf ("\nRobot Vision DFS Test Tool (%s %s)\n", __DATE__, __TIME__);
    for(int i=0; i<49; i++) putchar('-');
    printf("\n");
    printf ("Command: '%s'\n", argv0);
    printf ("Usage: %s [options]\n", exename.c_str());

	static const char* text =
        "   ----\t-----------\t\t-----------\n"
        "   Flag\tLong option\t\tDescription\n"
        "   ----\t-----------\t\t-----------\n"
		"    -m\t--mode={0|1|2|3}\t"    "running mode 0: CVP, 1: SW, 2: GPU, 3: Guided\n"
		"    -n\t--loop={count}\t\t"    "number of profiling loops in non-sequence mode\n"
		"    -d\t--dsp-min={0~255}\t"   "lower limit of disparity search range\n"
		"    -D\t--dsp-max={16~255}\t"  "how many levels of disparity search range, lower limit + levels = upper limit\n"
		"    -R\t--dsp-range\t\t"       "enables dynamic disparity range\n"
		"\t\t\t\t"                      "  e.g. alternately use the first and second half of the range\n"
		"    -r\t--rimage={filename}\t" "path of right input image\n"
		"    -l\t--limage={filename}\t" "path of left input image\n"
		"    -w\t--width={value}\t\t"   "width of input images\n"
		"    -h\t--height={value}\t"    "hight of input images\n"
		"    -s\t--stride={value}\t"    "stride of input images\n"
		"    -c\t--calib={filename}\t"  "calibration parameter file\n"
		"    -u\t--undist={0|1}\t"		"are undistortion&rectification needed?"
		"\t\t\t\t"						"0: input image pairs are undistorted and rectified, if stereo config file is provided, the parameters are rectified stereo parameters"
		"\t\t\t\t"						"1: input image pairs should be undistorted or rectified with config file, stereo config file must be provided"
		"    -f\t--fps={value}\t\t"     "desired fps\n"
		"    -o\t--format={0|1|2|3}\t"  "output format\n"
        "\t\t\t\t"                      "  0: disparity, 1: depth, 2: point cloud\n"
        "\t\t\t\t"                      "  3: point cloud fusion with left image\n"
        "    -v\t--verbose\t\t"         "verbose mode\n"
		"    -H\t--help\t\t\t"          "print this usage message\n\n";

    printf("%s", text);
    printf ("Example: %s -l left.png -r right.png -m 0 -d 1 -D 32 -c stereo_cal.yml -o 3\n", exename.c_str());

#ifdef WIN32
	printf(
		"!!! On windows, rectified image assumed,\n Usage: leftImageName "
		"rightImageName imageWidth imageHeight\n imageWidth & imageHeight valid "
		"for yuv format only\n");
#endif
}
/*----------------------------------------------------------------------------*/
static struct {
    const char *    long_symbol;
    const char      short_symbol;
    const char      option;
} arg_option_table[] =
{
    {"mode",        'm', 'y'},
    {"loop",        'n', 'y'},
    {"dsp-min",     'd', 'y'},
    {"dsp-max",     'D', 'y'},
    {"dsp-range",   'R', 'n'},
    {"rimage",      'r', 'y'},
    {"limage",      'l', 'y'},
    {"width",       'w', 'y'},
    {"height",      'h', 'y'},
    {"stride",      's', 'y'},
    {"calib",       'c', 'y'},
    {"fps",         'f', 'y'},
    {"format",      'o', 'y'},
    {"verbose",     'v', 'n'},
	{"undist",      'u', 'y'},
    {"help",        'h', 'n'},
    {0, 0}
};
/*----------------------------------------------------------------------------*/
int         arg_find_command(char short_symbol, string & cmd_string, char & option)
{
    for(int i=0; arg_option_table[i].long_symbol != nullptr; i++)
    {
        if (short_symbol == arg_option_table[i].short_symbol)
        {
            cmd_string = string(arg_option_table[i].long_symbol);
            option = arg_option_table[i].option;
            return short_symbol;
        }
    }
    return -1;
}
/*----------------------------------------------------------------------------*/
int         arg_find_command(string long_symbol, char & option)
{
    for(int i=0; arg_option_table[i].long_symbol != nullptr; i++)
    {
        if (long_symbol == arg_option_table[i].long_symbol)
        {
            option = arg_option_table[i].option;
            return arg_option_table[i].short_symbol;
        }
    }
    return -1;
}
/*----------------------------------------------------------------------------*/
int         arg_parse_command(int argc, const char * argv[], int & arg_index, string & cmd_string, string & opt_string)
{
    const char * str = argv[arg_index++];
    char option = 'n';
    int result = 0;

    if (str[0] == '-')
    {
        // if it's short switch?
        if (str[2] == '\0')
        {
            if ((result = arg_find_command(str[1], cmd_string, option)) < 0)
                return -1;

            if (option == 'n')
            {
                opt_string.clear();
                return result;
            }

            opt_string = string (arg_index < argc ? argv[arg_index] : "");

            if (opt_string.size() == 0 && option != 'o')
                return -1;

            if (option == 'o' && opt_string[0]=='-')
            {
                opt_string.clear();
                return result;
            }

            arg_index++;

            return result;
        }

        // if it's long switch?
        if (str[1] == '-' && str[2] != '\0')
        {
            cmd_string = string (str + 2);
            size_t pos = cmd_string.find('=');
            if (pos == string::npos)
            {
                opt_string.clear();

                result = arg_find_command(cmd_string, option);
                if (result > 0 && (option=='n' || option=='o'))
                    return result;

                cmd_string = string(str);
                return -1;
            }

            opt_string = cmd_string.substr(pos+1);
            cmd_string.erase(pos);

            return arg_find_command(cmd_string, option);
        }
        return -1;  // unknown command
    }

    cmd_string = string (str);
    opt_string.clear();

    return 0;
}
/*----------------------------------------------------------------------------*/
string      cct_addPostfix(const string & filepath, const string & postfix)
{
    string newfile = filepath;
    size_t extpos = newfile.rfind(".ply");

    if (extpos == string::npos)
        newfile += postfix + ".ply";
    else
        newfile.insert(extpos, postfix);
    return newfile;
}
/*----------------------------------------------------------------------------*/

int     parseCommandLine(int argc, const char* argv[])
{
    string cmdstring;
    string optstring;
    int arg_index = 1;
    int arg_count = 0;

	int c, mode, loops;
	bool strideNotSet=true;

    while(arg_index < argc)
    {
        int c = arg_parse_command(argc, argv, arg_index, cmdstring, optstring);

        if (gVerbose)
        {
            cout << "[ARG] code = " << c << ", cmdstring = '" << cmdstring << "', optstring = '" << optstring << "'" << endl;
        }

		switch (c) {
            case 'm':
                gRunningMode = (rvDFSMode)std::stoi(optstring);
                break;
            case 'n':
                gNumLoops = std::stoi(optstring);
                break;
            case 'd':
                gMinDisparity = std::stoi(optstring);
                break;
            case 'D':
                gLevelDisparity = std::stoi(optstring);
                break;
            case 'l':
                gLeftImage = optstring;
                break;
            case 'f':
                gFPS = std::stoi(optstring);
                break;
            case 'r':
                gRightImage = optstring;
                break;
            case 'h':
                gNumRows = std::stoi(optstring);
                break;
            case 'w':
                gNumCols = std::stoi(optstring);
                if (strideNotSet)
                {
                    gStride = gNumCols;
                }
                break;
            case 's':
                gStride = std::stoi(optstring);
                strideNotSet = false;
                break;
            case 'c':
                gStereoConfigFile = optstring;
                break;
            case 'o':
                gOutputFormat = std::stoi(optstring);
                if(gOutputFormat > 5)
                    gOutputFormat = 0;
                break;
            case 'R':
                gDynamicRange = true;
                break;
			case 'u':
				gDoRectification = std::stoi(optstring);
				break;
			case 'v':
				rvVersion();
				rvQueryOpenCLInfo();
				return 0;
            default:
                cout << "Error : can't handle '" << cmdstring << "', code='" << c << " ***" << endl;

            /* fall through */
            case 'H':
            case '?':
                printHelp(argv[0]);
                exit(1);
                break;
		}
        arg_count++;
	}

	if (gLeftImage.empty())
    {
        cout << "Error : left image is not specified!" << endl;
        printHelp(argv[0]);
        exit(1);
    }

	if (gStereoConfigFile.empty())
		gDoRectification = false;

	RV_DBG("leave parse Command Line");

	return arg_count;
}

void saveDisparityMap(std::string& fullFolder, float* disparityFloat, int width, int height)
{
	//save original disparity image
	cv::Mat disparityImageChar(height, width, CV_8UC1);
	cv::Mat disparityImageFloat(height, width, CV_32FC1);
	unsigned char* pDisparityChar = (unsigned char*)disparityImageChar.data;
	for (int ii = 0; ii < width * height; ++ii)
	{
		pDisparityChar[ii] = static_cast<unsigned char>(round(disparityFloat[ii]));
		if (gRunningMode == 2 && disparityFloat[ii] == 0.0)
			disparityFloat[ii] = INFINITY;		//gRunningMode 2 doesn't set INFINITY for disparity map
	}
	cv::Mat disp;
	disp = cv::Mat::zeros(height, width, CV_32FC1);
	memcpy(disp.data, disparityFloat, sizeof(float) * height * width);
	cv::imwrite(fullFolder + "/disparityOri.png", disp);

	//save colorized disparity image
	double min;
	double max;
	cv::minMaxIdx(disparityImageChar, &min, &max);
	double scale = 255. / (max - min);
	disparityImageChar.convertTo(disparityImageChar, CV_8UC1, scale, -min * scale);
	cv::Mat falseColorsMap;
	cv::applyColorMap(disparityImageChar, falseColorsMap, cv::COLORMAP_JET);
	cv::imwrite(fullFolder + "/disparity.png", falseColorsMap);
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
    dfs_parameter.filterHeight = 8;
    dfs_parameter.filterWidth = 16;
    dfs_parameter.disparity.minDisparity = gMinDisparity;
    dfs_parameter.disparity.numDisparityLevels = gLevelDisparity;
    dfs_parameter.doRectification = gDoRectification;
    dfs_parameter.doGpuRect = false;
	//For dynamic disparity range settings
	rvDFSDisparity dRange1,dRange2;

	rvDFSMode dfs_mode = rvDFSMode::RV_DFS_SPEED;
	if (gRunningMode == rvDFSMode::RV_DFS_CVP)
	{
		dfs_mode = rvDFSMode::RV_DFS_CVP;
	}
	else if (gRunningMode == rvDFSMode::RV_DFS_COVERAGE)
	{
		dfs_mode = rvDFSMode::RV_DFS_COVERAGE;
	}
	else if (gRunningMode == rvDFSMode::RV_DFS_SPEED)
	{
		dfs_mode = rvDFSMode::RV_DFS_SPEED;
		if (dfs_parameter.doRectification)
			dfs_parameter.doGpuRect = true;
	}
	else if (gRunningMode == rvDFSMode::RV_DFS_ACCURACY)
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
	RV_INFO("Elapsed time: %f ms", elapsed.count() * 1000 / gNumLoops);
	if (!gStereoConfigFile.empty())
	{
		PointCloudType pcl;
		rvDFS_Depth2PointCloud(dfs_handle, disparityMap->ptr<float>(), &pcl);
		std::string ply_file = fullFolder + ("/point_cloud.ply");
		dfs_test_tool::writePLYPointcloud(ply_file, pcl, disparityMap->cols, disparityMap->rows);
	}
#else // PROFILING
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
#endif // PROFILING
	rvDFS_Deinitialize(dfs_handle);
}

#ifdef DFS_CPP_STYLE_INTERFACE
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
	dfs_base->calculateDisparity(pLImg, pRImg, disparityMap->ptr<float>());
	dfs_base->deInitialize();
	
	dfs_base->initialize(gNumCols, gNumRows, gStride, dfs_parameter, stereo_parameter);
	rectified_stereo_parameter = dfs_base->getRectifiedCameraParameter();

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
			if (gStereoConfigFile.empty() || gOutputFormat == 0)
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
			else if (gOutputFormat == 1)
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
			else if (gOutputFormat == 2 || gOutputFormat == 4)	//point cloud
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
	// RV_ERR("Elapsed time: %f ms", elapsed.count() * 1000 / gNumLoops);
#ifdef GLOG_ENABLED
	LOG(INFO) << "Elapsed time (ms) is:   " << elapsed.count() * 1000 / gNumLoops;
#else
    cout << "[INFO] Elapsed time (ms) is:   " << elapsed.count() * 1000 / gNumLoops;
#endif
	if (gStereoConfigFile.empty() || gOutputFormat == 0)		//save disparity map, original and false color images, as well as pfm format which is designed by middlebury
	{
		saveDisparityMap(fullFolder, (float*)disparityMap->data, disparityMap->cols, disparityMap->rows);

#ifdef MIDDLEBERRY_EVAL
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
#endif // MIDDLEBURRY_EVAL
	}
	else if (gOutputFormat == 1)		//depth map, save rectified images as well
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

		if(gDoRectification)
		{
			//save rectified images
			dfs_base->getRectImages(rectLImg.ptr<uint8_t>(), rectRImg.ptr<uint8_t>());
			cv::imwrite(fullFolder + "/rightRectifiedImage.png", rectRImg);
			cv::imwrite(fullFolder + "/leftRectifiedImage.png", rectLImg);
		}

		//save raw depth image, assume the unit is centimeter
		cv::Mat depthImage(disparityMap->size(), CV_16UC1);
		unsigned short* pDepth = (unsigned short*)depthImage.data;
		float* pFloatDisparity = disparityMap->ptr<float>();
		for (int ii = 0; ii < disparityMap->cols * disparityMap->rows; ++ii)
		{
			pDepth[ii] = static_cast<unsigned short>(round(pFloatDisparity[ii]));
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
	else if (gOutputFormat == 2)		//point cloud
	{
		//save cloud point
		std::string ply_file = fullFolder + ("/point_cloud.ply");
		dfs_test_tool::writePLYPointcloud(ply_file, pcl, disparityMap->cols, disparityMap->rows);
	}
	else if (gOutputFormat == 3)	//point cloud color
	{
		//save cloud point fusion with gray scale left image
		std::string ply_file = fullFolder + ("/point_cloud_color.ply");
		dfs_test_tool::writePLYPointcloudColor(ply_file, pclColor, disparityMap->cols, disparityMap->rows);
	}
	else if (gOutputFormat == 4)	//point cloud and disparity map
	{
		//save cloud point
		std::string ply_file = fullFolder + ("/point_cloud.ply");
		dfs_test_tool::writePLYPointcloud(ply_file, pcl, disparityMap->cols, disparityMap->rows);
		//save disparity
		dfs_base->getDisparity((float*)disparityMap->data);
		saveDisparityMap(fullFolder, (float*)disparityMap->data, disparityMap->cols, disparityMap->rows);
	}
	else if (gOutputFormat == 5)	//point cloud fused with left image and disparity map
	{
		//save cloud point
		std::string ply_file = fullFolder + ("/point_cloud.ply");
		dfs_test_tool::writePLYPointcloudColor(ply_file, pclColor, disparityMap->cols, disparityMap->rows);
		//save disparity
		dfs_base->getDisparity((float*)disparityMap->data);
		saveDisparityMap(fullFolder, (float*)disparityMap->data, disparityMap->cols, disparityMap->rows);
	}

	if (gDoRectification) //if rectification is done by DFS, save rectified images
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
		
		dfs_base->getRectImages(rectLImg.ptr<uint8_t>(), rectRImg.ptr<uint8_t>());
		cv::imwrite(fullFolder + "/rightRectifiedImage.png", rectRImg);
		cv::imwrite(fullFolder + "/leftRectifiedImage.png", rectLImg);
	}

	return true;
}
#endif // DFS_CPP_STYLE_INTERFACE

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


int main(int argc, const char* argv[])
{
	int s;
	
	if (parseCommandLine(argc, argv) <= 0)
	{
	    printHelp(argv[0]);
        return 1;
	}

	rvStereoConfiguration stereo_parameter;
	if (!gStereoConfigFile.empty())
	{
		//load parameter files
		stereo_parameter = dfs_test_tool::importStereoCalData(gStereoConfigFile);
	}

	cv::Mat leftImage, rightImage;
	if(boost::filesystem::is_directory(boost::filesystem::path(gLeftImage)))
	{
		dfs_test_tool::processFolder(gLeftImage, gMinDisparity, gLevelDisparity,gDoRectification, gRunningMode, gOutputFormat, stereo_parameter);
	}
	else 
	{
		if (gRightImage.empty())
		{
			dfs_test_tool::readImage(gLeftImage.c_str(), leftImage, &gNumCols, &gNumRows, &gStride, true);
		}
		else
		{
			dfs_test_tool::readImage(gLeftImage.c_str(), leftImage, &gNumCols, &gNumRows, &gStride, false);
			dfs_test_tool::readImage(gRightImage.c_str(), rightImage, &gNumCols, &gNumRows, &gStride, false);
		}

		cv::Mat disp;
		disp = cv::Mat::zeros(gNumRows, gNumCols, CV_32FC1);

		// cv::Mat imgDisparity8U = cv::Mat(gNumRows, gNumCols, CV_8UC1);
	        // calDispWithSGBM(leftImage, rightImage, imgDisparity8U, gLevelDisparity, gNumLoops);

		std::string fullFolder = gLeftImage.c_str();
		s = fullFolder.find_last_of("\\");
		if (s < 0)
		{
			s = fullFolder.find_last_of("/");
		}
		if (s <= 0)
		{
			fullFolder = ".";
		}
		else
		{
			fullFolder.resize(s);
		}

#ifdef DFS_CPP_STYLE_INTERFACE
		if (!RunTestCpp(leftImage, rightImage, &disp, stereo_parameter, fullFolder))
                {
			cout << "[ERROR] '" << "Error in processing current image pair!";
		}
#else
		RunTestC(leftImage, rightImage, &disp, stereo_parameter, fullFolder);
#endif
	}

    cout << "[INFO] '" << gLeftImage << "' finished!" << endl;

	return 0;
}
