/*****************************************************************************
@copyright
Copyright (c) 2022-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#include <string.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#ifdef WIN32
#include "pfm.h"
#elif defined(MIDDLEBERRY_EVAL)
#include "imageLib.h"		//MiddEval3 SDK for middlebury dataset
#endif
#include "dfs_factory.h"
#include "rvDFS.h"
#include "dfs_test_tools.h"
#include <boost/filesystem.hpp>
#include "rvLog.h"

#ifdef PROFILING
#include <chrono>
#include <thread>
#include <mutex>
#endif

using namespace std;
namespace fs = boost::filesystem;
int RV_LOG_LEVEL = 1;
bool RV_STDERR_LOGGING = true;

// global variables
int gNumRows = 480, gNumCols = 640, gStride = 640;
int gNumLoops = 1;
int gMinDisparity = 1;
int gLevelDisparity = 32;
int gRunningMode = 1;
int gFPS = 0;
int gOutputFormat = 0;			//0: disparity, 1: depth, 2: point cloud, 3: point cloud fusion with left image
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

// void set_log() {
//   RV_LOG_SET_LEVEL(LEVEL_INFO);
//   RV_LOG_SET_FILE("/data/dfs_log/log_file.txt");
// }


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
    printf("\nRobot Vision DFS Test Tool (%s %s)\n", __DATE__, __TIME__);
    for(int i=0; i<49; i++) putchar('-');
    printf("\n");
    printf("Command: '%s'\n", argv0);
    printf("Usage: %s [options]\n", exename.c_str());

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
        "\t\t\t\t"                      "  3: point cloud fused with left image\n"
		"\t\t\t\t"                      "  4: disparity, depth and point cloud data.\n"
		"\t\t\t\t"                      "  5: disparity, detph, and point cloud color data. point cloud fused with left image and disparity image\n"
        "\t\t\t\t"                      "  6: point cloud data and disparity."
        "\t\t\t\t"                      "  7: point cloud color data and disparity."
        "    -v\t--verbose\t\t"         "verbose mode\n"
        "    -H\t--help\t\t\t"          "print this usage message\n\n";

    printf("%s", text);
    printf("Example: %s -l left.png -r right.png -m 0 -d 1 -D 32 -c stereo_cal.yml -o 3\n", exename.c_str());

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
    {"", 0, 0}
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
                gRunningMode = std::stoi(optstring);
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
                if(gOutputFormat > 7)
                    gOutputFormat = 0;
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

    if (gStereoConfigFile.empty())
        gDoRectification = false;
        
    return arg_count;
}


void RunTestC(cv::Mat& leftImage, cv::Mat& rightImage, const rvStereoCamera& stereo_parameter, std::string fullFolder)
{
    RV_INFO("Run C-style interface.");

    assert(gNumRows > 0 && gNumCols > 0);
    cv::Mat disparityMap = cv::Mat::zeros(gNumRows, gNumCols, CV_32FC1);

    rvDFSParameter dfs_parameter;
    dfs_parameter.filterHeight = 8;
    dfs_parameter.filterWidth = 16;
    dfs_parameter.disparity.minDisparity = gMinDisparity;
    dfs_parameter.disparity.numDisparityLevels = gLevelDisparity;
    dfs_parameter.doRectification = gDoRectification;
    dfs_parameter.doGpuRect = false;

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

    rvDFS* dfs_handle = rvDFS_Initialize(dfs_mode, gNumCols, gNumRows, gStride, dfs_parameter, stereo_parameter);
    if (dfs_handle == nullptr)
        return;
    rvStereoCamera rectified_stereo_parameter = rvDFS_GetRectifiedCameraParameter(dfs_handle);

    uint8_t* pLImg, * pRImg;
    pLImg = leftImage.ptr<uint8_t>();
    if (!gLeftImage.empty() && gRightImage.empty())
    {
        pRImg = nullptr;
    }
    else if(gLeftImage.empty() && !gRightImage.empty())
    {
       pLImg = rightImage.ptr<uint8_t>();
       pRImg = rightImage.ptr<uint8_t>() + gNumRows * gStride;
    }
    else
    {
        pRImg = rightImage.ptr<uint8_t>();
    }    
#ifdef PROFILING
    auto start = std::chrono::high_resolution_clock::now();
    if (gStereoConfigFile.empty())
    {
        for (int i = 0; i < gNumLoops; i++)
        {
            rvDFS_CalculateDisparity(dfs_handle, pLImg, pRImg, disparityMap.ptr<float>());
        }
    }
    else
    {
        for (int i = 0; i < gNumLoops; i++)
        {
            if(gOutputFormat==0)
            {
                rvDFS_CalculateDisparity(dfs_handle, pLImg, pRImg, disparityMap.ptr<float>());
            }
            else if(gOutputFormat==1)
            {
                rvDFS_CalculateDepth(dfs_handle, pLImg, pRImg, disparityMap.ptr<float>());
            }
            else
            {
                printf("TBD, std::vector in C?\n");
            }
        }
    }
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    printf("Elapsed time: %f ms\n", elapsed.count() * 1000 / gNumLoops);
    if (!gStereoConfigFile.empty())
    {
        PointCloudType pcl;
        rvDFS_Depth2PointCloud(dfs_handle, disparityMap.ptr<float>(), &pcl);
        std::string ply_file = fullFolder + ("/point_cloud.ply");
        dfs_test_tool::writePLYPointCloud(ply_file, pcl, disparityMap.cols, disparityMap.rows);
    }
#else // PROFILING
    if (gStereoConfigFile.empty())
    {
        if (!rvDFS_CalculateDisparity(dfs_handle, pLImg, pRImg, disparityMap.ptr<float>()))
            return;
    }
    else
    {
        if (!rvDFS_CalculateDepth(dfs_handle, pLImg, pRImg, disparityMap.ptr<float>()))
            return;
        PointCloudType pcl;
        if (!rvDFS_Depth2PointCloud(dfs_handle, disparityMap.ptr<float>(), &pcl))
            return;
        std::string ply_file = fullFolder + ("/point_cloud.ply");
        dfs_test_tool::writePLYPointCloud(ply_file, pcl, disparityMap.cols, disparityMap.rows);
    }
#endif // PROFILING
    rvDFS_Deinitialize(dfs_handle);
}


bool runTestCpp(cv::Mat& leftImage, cv::Mat& rightImage, const rvStereoCamera& stereo_parameter, std::string fullFolder)
{
    if (gStereoConfigFile.empty() && gDoRectification)
        return false; //must provide calibration parameters if want DFS to do rectification

    uint8_t* pLImg, * pRImg;
    pLImg = leftImage.ptr<uint8_t>();
    if (!gLeftImage.empty() && gRightImage.empty())
    {
        pRImg = nullptr;
    }
    else if(gLeftImage.empty() && !gRightImage.empty())
    {
       pLImg = rightImage.ptr<uint8_t>();
       pRImg = pLImg + gNumRows * gStride;
    }
    else
    {
        pRImg = rightImage.ptr<uint8_t>();
    }
    assert(gNumRows > 0 && gNumCols > 0 && pRImg != NULL);

    //prepare data for DFS results
    PointCloudType pcl;
    pcl.reserve(gNumCols * gNumRows * 3);
    PointCloudColorType pclColor;
    pclColor.reserve(gNumCols * gNumRows * 6);
    cv::Mat disparityMap = cv::Mat::zeros(gNumRows, gNumCols, CV_32FC1);
    cv::Mat depMap = cv::Mat::zeros(gNumRows, gNumCols, CV_32FC1);

    //prepare DFS parameters
    rvDFSParameter dfs_parameter;
    dfs_parameter.filterHeight = 9;
    dfs_parameter.filterWidth = 15;
    dfs_parameter.disparity.minDisparity = gMinDisparity;
    dfs_parameter.disparity.numDisparityLevels = gLevelDisparity;
    dfs_parameter.doRectification = gDoRectification;
    dfs_parameter.doGpuRect = false;

    //create DFS instance with the user specified mode
    std::mutex myMutex;
    myMutex.lock();
    rvDFSMode dfs_mode = rvDFSMode::RV_DFS_SPEED;
    if (gRunningMode == 0)
        dfs_mode = rvDFSMode::RV_DFS_CVP;
    else if (gRunningMode == 1)
        dfs_mode = rvDFSMode::RV_DFS_COVERAGE;
    else if (gRunningMode == 2)
    {
        dfs_mode = rvDFSMode::RV_DFS_SPEED;
        if (gDoRectification)
            dfs_parameter.doGpuRect = true;
    }
    else if (gRunningMode == 3)
        dfs_mode = rvDFSMode::RV_DFS_ACCURACY;
    myMutex.unlock();
    std::shared_ptr<rv_dfs::DFSBase> dfs_base = rv_dfs::CreateDFSbase(dfs_mode);
    if (dfs_base == nullptr)
        return false;

    //initialize the DFS instance with given parameters
    if (dfs_base->initialize(gNumCols, gNumRows, gStride, dfs_parameter, stereo_parameter) != true)
    {
        RV_ERR("dfs failed to initialize");
        return false;
    }

    //once initialized, user can use this API to get new intrinsics/extrinsics of rectified image pair without running DFS
    rvStereoCamera rectified_stereo_parameter = dfs_base->getRectifiedCameraParameter();
    
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < gNumLoops; i++)
    {
        if (gOutputFormat == 0)
            dfs_base->calculateDisparity(pLImg, pRImg, disparityMap.ptr<float>());
        else if (gOutputFormat == 1) // need depth image. "disparityMap" will be used here since it is "float" type
            dfs_base->calculateDepth(pLImg, pRImg, disparityMap.ptr<float>());
        else if (gOutputFormat == 2 || gOutputFormat == 6)
            dfs_base->calculatePointCloud(pLImg, pRImg, &pcl);
        else if( gOutputFormat == 3 || gOutputFormat == 7)
            dfs_base->calculatePointCloudColor(pLImg, pRImg, &pclColor);
        else if( gOutputFormat == 4)
            dfs_base->calculateDispDepthPointCloud(pLImg, pRImg, disparityMap.ptr<float>(), depMap.ptr<float>(), &pcl);
        else if( gOutputFormat == 5)
            dfs_base->calculateDispDepthPointCloudColor(pLImg, pRImg, disparityMap.ptr<float>(), depMap.ptr<float>(), &pclColor);
            
    }
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    cout << "[INFO] Elapsed time (ms) is:   " << elapsed.count() * 1000 / gNumLoops << std::endl;

    if(gOutputFormat == 0)
    {
        cv::imwrite(fullFolder + "/disparityOri.png", disparityMap);
        dfs_test_tool::saveColorizedDisparity(disparityMap, fullFolder + "/disparity.png");
	}
	else if (gOutputFormat == 1)
	{
        dfs_test_tool::saveDepthImage(disparityMap, fullFolder + "/depth.png");
        dfs_test_tool::saveColorizedDepthImage(disparityMap, fullFolder + "/depthColor.png");
	}
	else if (gOutputFormat == 2)
	{
		dfs_test_tool::writePLYPointCloud(fullFolder + "/point_cloud.ply", pcl, disparityMap.cols, disparityMap.rows);
	}
	else if (gOutputFormat == 3)
	{
		dfs_test_tool::writePLYPointCloudColor(fullFolder + "/point_cloud_color.ply", pclColor, disparityMap.cols, disparityMap.rows);
	}
    else if(gOutputFormat == 4) // save pc, disparity, depth
    {
		dfs_test_tool::writePLYPointCloud(fullFolder + "/point_cloud.ply", pcl, disparityMap.cols, disparityMap.rows);
        dfs_test_tool::saveColorizedDisparity(disparityMap, fullFolder + "/disparity.png");
        dfs_test_tool::saveColorizedDepthImage(depMap, fullFolder + "/depthColor.png");
    }
    else if(gOutputFormat == 5) // save pcc, disparity, depth
    {
		dfs_test_tool::writePLYPointCloudColor(fullFolder + "/point_cloud_color.ply", pclColor, disparityMap.cols, disparityMap.rows);
        dfs_test_tool::saveColorizedDisparity(disparityMap, fullFolder + "/disparity.png");
        dfs_test_tool::saveColorizedDepthImage(depMap, fullFolder + "/depthColor.png");
    }
	else if (gOutputFormat == 6) // pc, disparity
	{
		dfs_test_tool::writePLYPointCloud(fullFolder + "/point_cloud.ply", pcl, disparityMap.cols, disparityMap.rows);
        cv::Mat tempMap = cv::Mat::zeros(disparityMap.rows, disparityMap.cols, CV_32FC1);
		dfs_base->getDisparity((float*)tempMap.data);
        cv::imwrite(fullFolder + "/disparityOri.png", tempMap);
        dfs_test_tool::saveColorizedDisparity(tempMap, fullFolder + "/disparity.png");
	}
	else if (gOutputFormat == 7) // pcc, disparity
	{
		dfs_test_tool::writePLYPointCloudColor(fullFolder + "/point_cloud_color.ply", pclColor, disparityMap.cols, disparityMap.rows);
        cv::Mat tempMap = cv::Mat::zeros(disparityMap.rows, disparityMap.cols, CV_32FC1);
        dfs_base->getDisparity((float*)tempMap.data);
        cv::imwrite(fullFolder + "/disparityOri.png", tempMap);
        dfs_test_tool::saveColorizedDisparity(tempMap, fullFolder + "/disparity.png");
	}

    //if rectification is done by DFS, save rectified images
	if (gDoRectification) 
	{
		cv::Mat rectLImg, rectRImg;
		if (gRunningMode == 3)
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

    //finally de-initialize the DFS instance
    dfs_base->deInitialize();

	return true;
}


void readRectifiedPara(rvStereoCamera& rectified_stereo_parameter, const std::string& file)
{
    //load parameter files
    cv::FileStorage fs(file, cv::FileStorage::READ);
    if (!fs.isOpened())
        return;
    cv::Mat P1, P2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    //rectified_stereo_parameter
    double disparity_to_depth_factor_ = P2.at<double>(0, 3);
    double rectified_focal_length = P1.at<double>(0, 0);
    //update rectified_stereo_parameter_
    for (int k = 0; k < 2; ++k) {
        rectified_stereo_parameter.camera[k].focalLength[0] = float32_t (rectified_focal_length);
        rectified_stereo_parameter.camera[k].focalLength[1] = float32_t (rectified_focal_length);
        rectified_stereo_parameter.camera[k].principalPoint[0] = float32_t (P1.at<double>(0, 2));
        rectified_stereo_parameter.camera[k].principalPoint[1] = float32_t (P1.at<double>(1, 2));
        for (int i = 0; i < 8; ++i)
            rectified_stereo_parameter.camera[k].distortion[i] = 0.0;
    }
    for (int i = 0; i < 3; ++i)
    {
        rectified_stereo_parameter.rotation[i] = 0.0;
        rectified_stereo_parameter.translation[i] = 0.0;
    }
    if (rectified_focal_length != 0.0)
        rectified_stereo_parameter.translation[0] = float32_t (disparity_to_depth_factor_ / rectified_focal_length);
}


bool RunFixedFPSTest(cv::Mat& leftImage, cv::Mat& rightImage, const rvStereoCamera& stereo_parameter, std::string fullFolder)
{
    gRunningMode = 2; // only for GPU mode
    if (gStereoConfigFile.empty() && gDoRectification)
        return false; //must provide calibration parameters if want DFS to do rectification

    assert(gFPS > 0);
    assert(gNumRows > 0 && gNumCols > 0);
    cv::Mat disparityMap = cv::Mat::zeros(gNumRows, gNumCols, CV_32FC1);
    cv::Mat depMap = cv::Mat::zeros(gNumRows, gNumCols, CV_32FC1);

    rvDFSParameter dfs_parameter;
    dfs_parameter.filterHeight = 9;
    dfs_parameter.filterWidth = 15;
    dfs_parameter.disparity.minDisparity = gMinDisparity;
    dfs_parameter.disparity.numDisparityLevels = gLevelDisparity;
    dfs_parameter.doRectification = gDoRectification;
    dfs_parameter.doGpuRect = false;

    PointCloudType pcl;
    pcl.reserve(gNumCols * gNumRows * 3);
    PointCloudColorType pclColor;
    pclColor.reserve(gNumCols * gNumRows * 6);

    rvDFSMode dfs_mode = rvDFSMode::RV_DFS_SPEED;
    if (gDoRectification)
        dfs_parameter.doGpuRect = true;
    int outputFormat = gOutputFormat;

    std::shared_ptr<rv_dfs::DFSBase> dfs_base = rv_dfs::CreateDFSbase(dfs_mode);
    if (dfs_base == nullptr)
        return false;

    uint8_t* pLImg, * pRImg;
    pLImg = leftImage.ptr<uint8_t>();
    if (!gLeftImage.empty() && gRightImage.empty())  // only left_img
    {
        pRImg = nullptr;
    }
    else if(gLeftImage.empty() && !gRightImage.empty())  // only right_img
    {
       pLImg = rightImage.ptr<uint8_t>();
       pRImg = rightImage.ptr<uint8_t>() + gNumRows * gStride;
    }
    else // both left and right imgs
    {
        pRImg = rightImage.ptr<uint8_t>();
    }
    
    if (dfs_base->initialize(gNumCols, gNumRows, gStride, dfs_parameter, stereo_parameter) != true)
    {
        RV_ERR("dfs failed to initialize");
        return false;
    }

    auto start = std::chrono::high_resolution_clock::now();
    auto durFrame = std::chrono::duration<double>(1.0 / gFPS);
    auto thres = std::chrono::duration<double>(0.000001);
    auto tsPrev = start;
    for (int i = 0; i < gNumLoops; i++)
    {
        if (outputFormat == 0)
            dfs_base->calculateDisparity(pLImg, pRImg, disparityMap.ptr<float>());
        else if (gOutputFormat == 1)
            dfs_base->calculateDepth(pLImg, pRImg, disparityMap.ptr<float>());
        else if (gOutputFormat == 2 || gOutputFormat == 6)
            dfs_base->calculatePointCloud(pLImg, pRImg, &pcl);
        else if( gOutputFormat == 3 || gOutputFormat == 7)
            dfs_base->calculatePointCloudColor(pLImg, pRImg, &pclColor);
        else if( gOutputFormat == 4)
            dfs_base->calculateDispDepthPointCloud(pLImg, pRImg, disparityMap.ptr<float>(), depMap.ptr<float>(), &pcl);
        else if( gOutputFormat == 5)
            dfs_base->calculateDispDepthPointCloudColor(pLImg, pRImg, disparityMap.ptr<float>(), depMap.ptr<float>(), &pclColor);

        auto ts = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dur = durFrame - (ts - tsPrev);
        if (dur > thres)
            std::this_thread::sleep_for(dur);
        tsPrev = std::chrono::high_resolution_clock::now();
    }
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    cout << "[INFO] Elapsed time (ms) is:   " << elapsed.count() * 1000 / gNumLoops << std::endl;

    if (gOutputFormat == 0) //save disparity map (original and colorized)
    {
        cv::imwrite(fullFolder + "/disparityOri.png", disparityMap);
        dfs_test_tool::saveColorizedDisparity(disparityMap, fullFolder + "/disparity.png");
    }
    else if (gOutputFormat == 1) //save depth map
    {
        dfs_test_tool::saveDepthImage(disparityMap, fullFolder + "/depth.png");
        dfs_test_tool::saveColorizedDepthImage(disparityMap, fullFolder + "/depthColor.png");
    }
    else if (gOutputFormat == 2)
    {
        dfs_test_tool::writePLYPointCloud(fullFolder + "/point_cloud.ply", pcl, disparityMap.cols, disparityMap.rows);
    }
    else if (gOutputFormat == 3)
    {
        dfs_test_tool::writePLYPointCloudColor(fullFolder + "/point_cloud_color.ply", pclColor, disparityMap.cols, disparityMap.rows);
    }
    else if(gOutputFormat == 4) // save pc, disparity, depth
    {
		dfs_test_tool::writePLYPointCloud(fullFolder + "/point_cloud.ply", pcl, disparityMap.cols, disparityMap.rows);
        dfs_test_tool::saveColorizedDisparity(disparityMap, fullFolder + "/disparity.png");
        dfs_test_tool::saveColorizedDepthImage(depMap, fullFolder + "/depthColor.png");
    }
    else if(gOutputFormat == 5) // save pcc, disparity, depth
    {
		dfs_test_tool::writePLYPointCloudColor(fullFolder + "/point_cloud_color.ply", pclColor, disparityMap.cols, disparityMap.rows);
        dfs_test_tool::saveColorizedDisparity(disparityMap, fullFolder + "/disparity.png");
        dfs_test_tool::saveColorizedDepthImage(depMap, fullFolder + "/depthColor.png");
    }
    else if (gOutputFormat == 6) // save pc, disparity
    {
        dfs_test_tool::writePLYPointCloud(fullFolder + "/point_cloud.ply", pcl, disparityMap.cols, disparityMap.rows);
        cv::Mat tempMap = cv::Mat::zeros(disparityMap.rows, disparityMap.cols, CV_32FC1);
        dfs_base->getDisparity((float*)tempMap.data);
        cv::imwrite(fullFolder + "/disparityOri.png", tempMap);
        dfs_test_tool::saveColorizedDisparity(tempMap, fullFolder + "/disparity.png");
    }
    else if (gOutputFormat == 7) // save pcc, disparity
    {
        dfs_test_tool::writePLYPointCloudColor(fullFolder + "/point_cloud_color.ply", pclColor, disparityMap.cols, disparityMap.rows);
        cv::Mat tempMap = cv::Mat::zeros(disparityMap.rows, disparityMap.cols, CV_32FC1);
        dfs_base->getDisparity((float*)tempMap.data);
        cv::imwrite(fullFolder + "/disparityOri.png", tempMap);
        dfs_test_tool::saveColorizedDisparity(tempMap, fullFolder + "/disparity.png");
    }

    dfs_base->deInitialize();

    return true;
}

bool runNewAPIsTest(cv::Mat& leftImage, cv::Mat& rightImage, const rvStereoCamera& stereo_param, std::string& fullFolder)
{
    // user setting
    rvDFSMode dfs_mode = rvDFSMode::RV_DFS_SPEED;
    gDoRectification = false;

    rvDFSParameter dfs_param;
    dfs_param.filterHeight= 9;
    dfs_param.filterWidth = 15;
    dfs_param.disparity.minDisparity = gMinDisparity;
    dfs_param.disparity.numDisparityLevels = gLevelDisparity;
    dfs_param.doRectification = gDoRectification;
    dfs_param.doGpuRect = false;
    if( gDoRectification) dfs_param.doGpuRect = true;
    // print info
    printf("minDisparity: %d, numDisparityLevels: %d\n", gMinDisparity, gLevelDisparity);
    printf("doRectification: %d, doGpuRect: %d\n", dfs_param.doRectification, dfs_param.doGpuRect);

    int pixel_num = gNumRows* gNumCols;
    cv::Mat dispMat = cv::Mat::zeros(gNumRows, gNumCols, CV_32FC1);
    cv::Mat depMat = cv::Mat::zeros(gNumRows, gNumCols, CV_32FC1);
    PointCloudType pcl;
    PointCloudColorType pclColor;
    pcl.reserve(pixel_num * 3);
    pclColor.reserve(pixel_num* 6);

    uint8_t* imgl;
    uint8_t* imgr;
    imgl = leftImage.ptr<uint8_t>();
    imgr = rightImage.ptr<uint8_t>();

    std::shared_ptr<rv_dfs::DFSBase> dfs_base = rv_dfs::CreateDFSbase(dfs_mode);
    if( dfs_base->initialize(gNumCols, gNumRows, gStride, dfs_param, stereo_param)== false){ RV_ERR("dfs init failed"); return false; }

    // dfs_base->calculateDisparity(imgl, imgr, dispMat.ptr<float>());
    // dfs_base->calculateDepth(imgl, imgr, depMat.ptr<float>());
    // dfs_base->calculatePointCloud(imgl, imgr, &pcl);
    // dfs_base->calculatePointCloudColor(imgl, imgr, &pclColor);

    dfs_base->calculateDispDepthPointCloudColor(imgl, imgr, dispMat.ptr<float>(), depMat.ptr<float>(), &pclColor);
    dfs_test_tool::writePLYPointCloudColor(fullFolder + "/pclColor.ply", pclColor, gNumCols, gNumRows);
    
    dfs_base->calculateDispDepthPointCloud(imgl, imgr, dispMat.ptr<float>(), depMat.ptr<float>(), &pcl);
    dfs_test_tool::writePLYPointCloud(fullFolder + "/pcl.ply", pcl, gNumCols, gNumRows);

    dfs_test_tool::saveColorizedDisparity(dispMat, fullFolder+ "/disparityColor.png");
    dfs_test_tool::saveColorizedDepthImage(depMat, fullFolder + "/depthColor.png");
    dfs_test_tool::saveDepthImage(depMat, fullFolder + "/depth.png");

    dfs_base->deInitialize();
    return true;
}

bool runDynamicRangeTest(cv::Mat& leftImage, cv::Mat& rightImage, const rvStereoCamera& stereo_parameter, std::string fullFolder)
{
    if (gStereoConfigFile.empty() && gDoRectification)
        return false; //must provide calibration parameters if want DFS to do rectification

    assert(gNumRows > 0 && gNumCols > 0);
    cv::Mat disparityMap = cv::Mat::zeros(gNumRows, gNumCols, CV_32FC1);
    cv::Mat depMap = cv::Mat::zeros(gNumRows, gNumCols, CV_32FC1);

    rvDFSParameter dfs_parameter;
    dfs_parameter.filterHeight = 9;
    dfs_parameter.filterWidth = 15;
    dfs_parameter.disparity.minDisparity = gMinDisparity;
    dfs_parameter.disparity.numDisparityLevels = gLevelDisparity;
    dfs_parameter.doRectification = gDoRectification;
    dfs_parameter.doGpuRect = false;

    rvDFSDisparity dRange1, dRange2;		//For dynamic disparity range settings
    PointCloudType pcl;
    pcl.reserve(gNumCols * gNumRows * 3);
    PointCloudColorType pclColor;
    pclColor.reserve(gNumCols * gNumRows * 6);

    dRange1.minDisparity = gMinDisparity;
    dRange1.numDisparityLevels = gLevelDisparity / 2;
    dRange2.minDisparity = gMinDisparity + dRange1.numDisparityLevels;
    dRange2.numDisparityLevels = gLevelDisparity - gLevelDisparity / 2;

    std::mutex myMutex;
    myMutex.lock();
    rvDFSMode dfs_mode = rvDFSMode::RV_DFS_SPEED;
    if (gRunningMode == 0)
        dfs_mode = rvDFSMode::RV_DFS_CVP;
    else if (gRunningMode == 1)
        dfs_mode = rvDFSMode::RV_DFS_COVERAGE;
    else if (gRunningMode == 2)
    {
        dfs_mode = rvDFSMode::RV_DFS_SPEED;
        if (gDoRectification)
            dfs_parameter.doGpuRect = true;
    }
    else if (gRunningMode == 3)
        dfs_mode = rvDFSMode::RV_DFS_ACCURACY;
    int outputFormat = gOutputFormat;
    myMutex.unlock();


    std::shared_ptr<rv_dfs::DFSBase> dfs_base = rv_dfs::CreateDFSbase(dfs_mode);
    if (dfs_base == nullptr)
        return false;

    uint8_t* pLImg, * pRImg;
    pLImg = leftImage.ptr<uint8_t>();
    if (!gLeftImage.empty() && gRightImage.empty()) // only left_img
    {
        pRImg = nullptr;
    }
    else if(gLeftImage.empty() && !gRightImage.empty()) // only right_img
    {
       pLImg = rightImage.ptr<uint8_t>();
       pRImg = rightImage.ptr<uint8_t>() + gNumRows * gStride;
    }
    else  // both left and right imgs
    {
        pRImg = rightImage.ptr<uint8_t>();
    }

    rvStereoCamera rectified_stereo_parameter = dfs_base->getRectifiedCameraParameter();
    if (dfs_base->initialize(gNumCols, gNumRows, gStride, dfs_parameter, stereo_parameter) != true)
    {
        RV_ERR("dfs failed to initialize");
        return false;
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < gNumLoops; i++)
    {
        if (outputFormat == 0)
        {
            if (i % 2 == 0)
                dfs_base->calculateDisparity(pLImg, pRImg, disparityMap.ptr<float>(), &dRange1);
            else
                dfs_base->calculateDisparity(pLImg, pRImg, disparityMap.ptr<float>(), &dRange2);
        }
        else if (outputFormat == 1)
        {
            if (i % 2 == 0)
                dfs_base->calculateDepth(pLImg, pRImg, disparityMap.ptr<float>(), &dRange1);
            else
                dfs_base->calculateDepth(pLImg, pRImg, disparityMap.ptr<float>(), &dRange2);
        }
        else if (outputFormat == 2 || outputFormat == 6)
        {
            if (i % 2 == 0)
                dfs_base->calculatePointCloud(pLImg, pRImg, &pcl, &dRange1);
            else
                dfs_base->calculatePointCloud(pLImg, pRImg, &pcl, &dRange2);
        }
        else if (outputFormat == 3 || outputFormat == 7)
        {
            if (i % 2 == 0)
                dfs_base->calculatePointCloudColor(pLImg, pRImg, &pclColor, &dRange1);
            else
                dfs_base->calculatePointCloudColor(pLImg, pRImg, &pclColor, &dRange2);
        }
        else if(outputFormat == 4) // disparity, depth, pc
        {
            if (i % 2 == 0)
                dfs_base->calculateDispDepthPointCloud(pLImg, pRImg, disparityMap.ptr<float>(), depMap.ptr<float>(), &pcl, &dRange1);
            else
                dfs_base->calculateDispDepthPointCloud(pLImg, pRImg, disparityMap.ptr<float>(), depMap.ptr<float>(), &pcl, &dRange2);
        }
        else if(outputFormat == 5) // disparity, depth, pcc
        {
            if (i % 2 == 0)
                dfs_base->calculateDispDepthPointCloudColor(pLImg, pRImg, disparityMap.ptr<float>(), depMap.ptr<float>(), &pclColor, &dRange1);
            else
                dfs_base->calculateDispDepthPointCloudColor(pLImg, pRImg, disparityMap.ptr<float>(), depMap.ptr<float>(), &pclColor, &dRange2);
        }
    }
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    cout << "[INFO] Elapsed time (ms) is:   " << elapsed.count() * 1000 / gNumLoops << std::endl;

    if (gOutputFormat == 0)
    {
        cv::imwrite(fullFolder + "/disparityOri.png", disparityMap);
        dfs_test_tool::saveColorizedDisparity(disparityMap, fullFolder + "/disparity.png");
    }
    else if (gOutputFormat == 1)
    {
        dfs_test_tool::saveDepthImage(disparityMap, fullFolder + "/depth.png");
        dfs_test_tool::saveColorizedDepthImage(disparityMap, fullFolder + "/depthColor.png");
    }
    else if (gOutputFormat == 2)
    {
        dfs_test_tool::writePLYPointCloud(fullFolder + "/point_cloud.ply", pcl, disparityMap.cols, disparityMap.rows);
    }
    else if (gOutputFormat == 3)
    {
        dfs_test_tool::writePLYPointCloudColor(fullFolder + "/point_cloud_color.ply", pclColor, disparityMap.cols, disparityMap.rows);
    }
    else if(gOutputFormat == 4) // save pc, disparity, depth
    {
		dfs_test_tool::writePLYPointCloud(fullFolder + "/point_cloud.ply", pcl, disparityMap.cols, disparityMap.rows);
        dfs_test_tool::saveColorizedDisparity(disparityMap, fullFolder + "/disparity.png");
        dfs_test_tool::saveColorizedDepthImage(depMap, fullFolder + "/depthColor.png");
    }
    else if(gOutputFormat == 5) // save pcc, disparity, depth
    {
		dfs_test_tool::writePLYPointCloudColor(fullFolder + "/point_cloud_color.ply", pclColor, disparityMap.cols, disparityMap.rows);
        dfs_test_tool::saveColorizedDisparity(disparityMap, fullFolder + "/disparity.png");
        dfs_test_tool::saveColorizedDepthImage(depMap, fullFolder + "/depthColor.png");
    }
    else if (gOutputFormat == 6) // save pc, disparity
    {
        dfs_test_tool::writePLYPointCloud(fullFolder + "/point_cloud.ply", pcl, disparityMap.cols, disparityMap.rows);
        cv::Mat tempMap = cv::Mat::zeros(disparityMap.rows, disparityMap.cols, CV_32FC1);
        dfs_base->getDisparity((float*)tempMap.data);
        cv::imwrite(fullFolder + "/disparityOri.png", tempMap);
        dfs_test_tool::saveColorizedDisparity(tempMap, fullFolder + "/disparity.png");
    }
    else if (gOutputFormat == 7) // save pcc, disparity
    {
        dfs_test_tool::writePLYPointCloudColor(fullFolder + "/point_cloud_color.ply", pclColor, disparityMap.cols, disparityMap.rows);
        cv::Mat tempMap = cv::Mat::zeros(disparityMap.rows, disparityMap.cols, CV_32FC1);
        dfs_base->getDisparity((float*)tempMap.data);
        cv::imwrite(fullFolder + "/disparityOri.png", tempMap);
        dfs_test_tool::saveColorizedDisparity(tempMap, fullFolder + "/disparity.png");
    }

    dfs_base->deInitialize();

    return true;
}


bool runMiddleburyTest(cv::Mat& leftImage, cv::Mat& rightImage, const rvStereoCamera& stereo_parameter, std::string fullFolder)
{
    gOutputFormat = 0; // for Middlebury, only need to generate disparity image

    uint8_t* pLImg = leftImage.ptr<uint8_t>();
    uint8_t* pRImg = rightImage.ptr<uint8_t>();
    assert(gNumRows > 0 && gNumCols > 0 && pLImg != NULL && pRImg != NULL);

    //prepare data for DFS results
    cv::Mat disparityMap = cv::Mat::zeros(gNumRows, gNumCols, CV_32FC1);

    //prepare DFS parameters
    rvDFSParameter dfs_parameter;
    dfs_parameter.filterHeight = 9;
    dfs_parameter.filterWidth = 15;
    dfs_parameter.disparity.minDisparity = gMinDisparity;
    dfs_parameter.disparity.numDisparityLevels = gLevelDisparity;
    dfs_parameter.doRectification = false; // Middlebury images are already rectified
    dfs_parameter.doGpuRect = false;

    //create DFS instance with the user specified mode
    std::mutex myMutex;
    myMutex.lock();
    rvDFSMode dfs_mode = rvDFSMode::RV_DFS_SPEED;
    if (gRunningMode == 0)
        dfs_mode = rvDFSMode::RV_DFS_CVP;
    else if (gRunningMode == 1)
        dfs_mode = rvDFSMode::RV_DFS_COVERAGE;
    else if (gRunningMode == 2)
        dfs_mode = rvDFSMode::RV_DFS_SPEED;
    else if (gRunningMode == 3)
        dfs_mode = rvDFSMode::RV_DFS_ACCURACY;
    myMutex.unlock();
    std::shared_ptr<rv_dfs::DFSBase> dfs_base = rv_dfs::CreateDFSbase(dfs_mode);
    if (dfs_base == nullptr)
        return false;

    //initialize the DFS instance with given parameters
    if (dfs_base->initialize(gNumCols, gNumRows, gStride, dfs_parameter, stereo_parameter) != true)
    {
        RV_ERR("dfs failed to initialize");
        return false;
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < gNumLoops; i++)
    {
        if (gOutputFormat == 0)
            dfs_base->calculateDisparity(pLImg, pRImg, disparityMap.ptr<float>());
    }
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    cout << "[INFO] Elapsed time (ms) is:   " << elapsed.count() * 1000 / gNumLoops << std::endl;

    if (gOutputFormat == 0)		//save disparity map, original and false color images, as well as pfm format which is designed by middlebury
    {
        //save original disparity image
        cv::imwrite(fullFolder + "/disparityOri.png", disparityMap);
        //generate and save colorized disparity image
        dfs_test_tool::saveColorizedDisparity(disparityMap, fullFolder + "/disparity.png");

        //save "pfm" image for evaluation
        std::string pfmPath = fullFolder + "/disp0FCVF.pfm";
#ifdef WIN32
        WriteFilePFM((float*)disparityMap.data, disparityMap.cols, disparityMap.rows, pfmPath.c_str());
#elif defined(MIDDLEBERRY_EVAL)
        CShape sh(disparityMap.cols, disparityMap.rows, 1);
        CFloatImage fdisp;
        fdisp.ReAllocate(sh, (float*)disparityMap.data, false, sh.width * sizeof(float));
        WriteFilePFM(fdisp, pfmPath.c_str(), (float)(1.0 / 255.0));
#endif
    }    

    //finally de-initialize the DFS instance
    dfs_base->deInitialize();

    return true;
}


int main(int argc, const char* argv[])
{	
	if (parseCommandLine(argc, argv) <= 0)
	{
	    printHelp(argv[0]);
        return 1;
    }

    fs::path abs_path = fs::system_complete( gLeftImage);
    gLeftImage = abs_path.string();
    // printf("left img path: %s\n", gLeftImage.c_str());

	rvStereoCamera stereo_parameter;
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

        if (gRightImage.empty() && !gLeftImage.empty())
        {
            dfs_test_tool::readImage(gLeftImage.c_str(), leftImage, &gNumCols, &gNumRows, &gStride, true);
        }
        else if(gLeftImage.empty() && !gRightImage.empty())
        {
            //this is a top down image, so total height is gNumRows*2
            int inputHeight = gNumRows*2;
            dfs_test_tool::readImage(gRightImage.c_str(), rightImage, &gNumCols, &inputHeight, &gStride, false);
        }
		else
		{
			dfs_test_tool::readImage(gLeftImage.c_str(), leftImage, &gNumCols, &gNumRows, &gStride, false);
			dfs_test_tool::readImage(gRightImage.c_str(), rightImage, &gNumCols, &gNumRows, &gStride, false);
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
        bool res = runTestCpp(leftImage, rightImage, stereo_parameter, fullFolder);
        if (!res)
            RV_ERR("Error in processing current image pair!");
#else
        RunTestC(leftImage, rightImage, stereo_parameter, fullFolder);
#endif
	}

    RV_INFO("%s finished!",gLeftImage.c_str());

    return 0;
}
