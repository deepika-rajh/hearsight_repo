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
#include "dfs_factory.h"
#include "rvDFS.h"
#include "dfs_test_tools.h"
#include "rvLog.h"
#include <filesystem>
#include <thread>

using namespace std;
namespace fs = std::filesystem;
int RV_LOG_LEVEL = 1;
bool RV_STDERR_LOGGING = true;

// global variables
int gNumRows = 480, gNumCols = 640, gStride = 640;
int gNumLoops = 1;
float gMinDisparity = 1.0;
float gLevelDisparity = 64.0;
int gRunningMode = 1;
int gFPS = 0;
int gOutputFormat = 0;            //0: disparity, 1: depth, 2: point cloud, 3: point cloud fusion with left image
string gLeftImage = "";
string gRightImage = "";
string gStereoConfigFile = "";
bool gDoRectification = false;    //False: input image pairs are undistorted and rectified, if stereo config file is provided, the parameters are rectified stereo parameters
                                //True: input image pairs should be undistorted or rectified with config file, stereo config file must be provided
int gRoiStartX = 0;
int gRoiStartY = 0;
int gRoiWidth = 0;
int gRoiHeight = 0;
bool gVerbose = false;
bool gUseDepth=false;
bool gDynamicDisp=false;
int gResizeRows = 0;
int gResizeCols = 0;
bool gHwCmdSet = false;

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
        "    -m\t--mode={0|1|2|3|4}\t"      "running mode 0: CVP, 1: Coverage, 2: Speed, 3: Accuracy, 4: Balance\n"
        "    -n\t--loop={count}\t\t"        "number of profiling loops in non-sequence mode\n"
        "    -d\t--d-min={0~240}\t\t"       "lower limit of disparity search range\n"
        "    -D\t--d-level={16~255}\t"      "how many levels of disparity search range, lower limit + levels = upper limit\n"
        "    -r\t--rimage={filename}\t"     "path of right input image\n"
        "    -l\t--limage={filename}\t"     "path of left input image\n"
        "    -w\t--width={value}\t\t"       "raw width of input images\n"
        "    -h\t--height={value}\t"        "raw height of input images\n"
        "    -s\t--stride={value}\t"        "stride of input images\n"
        "    -W\t--resize_width={value}\t"  "downsampling input images to new width\n"
        "    -H\t--resize_height={value}\t"  "downsampling input images to new height\n"
        "    -c\t--calib={filename}\t"      "calibration parameter file\n"
        "    -u\t--undist={0|1}\t\t"        "are undistortion&rectification needed?\n"
        "\t\t\t\t"                            "0: input image pairs are undistorted and rectified, if stereo config file is provided, the parameters are rectified stereo parameters\n"
        "\t\t\t\t"                            "1: input image pairs should be undistorted or rectified with config file, stereo config file must be provided\n"
        "    -f\t--fps={value}\t\t"         "desired fps\n"
        "    -o\t--format={0~8}\t\t"        "output format\n"
        "\t\t\t\t"                          "  0: disparity, 1: depth, 2: point cloud\n"
        "\t\t\t\t"                          "  3: point cloud fused with left image\n"
        "\t\t\t\t"                          "  4: disparity, depth, point cloud & point cloud color.\n"
        "\t\t\t\t"                          "  5: point cloud data and depth.\n"
        "\t\t\t\t"                          "  6: point cloud color data and disparity.\n"
        "\t\t\t\t"                          "  7: depth and point cloud\n"
        "\t\t\t\t"                          "  8: depth, point cloud and rectified images.\n"
        "    -R\t--roi={x,y,width,height}"  "ROI upper left corner x,y, ROI width and height\n"
        "    -Y\t--dynamicDisp\t\t"         "Disparity range changes online demo\n"
        "    -U\t--useDepth\t\t"            "-d and -D is depth and unit should align with extrinsic parameters, otherwise is disparity\n"
        "    -v\t--verbose\t\t"             "verbose mode\n"
        "    -H\t--help\t\t\t"              "print this usage message\n\n";

    printf("%s", text);
    printf("Example: %s -l left.png -r right.png -m 0 -d 1 -D 32 -c stereo_cal.yml -o 3 -R 64x32x128x32\n", exename.c_str());

#ifdef WIN32
    printf(
        "!!! On windows, rectified image assumed,\n Usage: leftImageName "
        "rightImageName imageWidth imageHeight\n imageWidth & imageHeight valid "
        "for yuv format only\n");
#endif
}
/*----------------------------------------------------------------------------*/
static struct 
{
    const char *    long_symbol;
    const char      short_symbol;
    const char      option;
} arg_option_table[] =
{
    {"mode",        'm', 'y'},
    {"loop",        'n', 'y'},
    {"d-min",       'd', 'y'},
    {"d-level",     'D', 'y'},
    {"rimage",      'r', 'y'},
    {"limage",      'l', 'y'},
    {"width",       'w', 'y'},
    {"height",      'h', 'y'},
    {"stride",      's', 'y'},
    {"resize-width",'W', 'y'},
    {"resize-height",'H', 'y'},
    {"calib",       'c', 'y'},
    {"fps",         'f', 'y'},
    {"format",      'o', 'y'},
    {"verbose",     'v', 'n'},
    {"undist",      'u', 'y'},
    {"roi",         'R', 'y'},
    {"useDepth",    'U', 'n'},
    {"dynamicDisp", 'Y', 'n'},
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
                if (result > 0)// && (option=='n' || option=='o'))
                    return result;

                cmd_string = string(str);
                // return -1;
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

        switch (c) 
        {
            case 'm':
                gRunningMode = std::stoi(optstring);
                break;
            case 'n':
                gNumLoops = std::stoi(optstring);
                break;
            case 'd':
                gMinDisparity = std::stof(optstring);
                break;
            case 'D':
                gLevelDisparity = std::stof(optstring);
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
            case 'H':
                gResizeRows = std::stoi(optstring);
                gHwCmdSet = true;
                break;
            case 'W':
                gResizeCols = std::stoi(optstring);
                gHwCmdSet = true;
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
                if(gOutputFormat > 8)
                {
                    RV_ERR("output format requested is out of range");
                    printHelp(argv[0]);
                    exit(1);
                }
                break;
            case 'u':
                gDoRectification = std::stoi(optstring);
                break;
            case 'R':
                {
                    std::string roi = optstring;
                    size_t roiData[4];
                    size_t pos = 0;
                    size_t i=0;
                    while ((pos = roi.find('x')) != std::string::npos)
                    {
                        roiData[i] = std::stoi(roi.substr(0, pos));
                        roi.erase(0, pos + 1);
                        i++;
                    }
                    gRoiStartX = roiData[0];
                    gRoiStartY = roiData[1];
                    gRoiWidth = roiData[2];
                    gRoiHeight = std::stoi(roi);
                }
                break;
            case 'U':
                gUseDepth=true;
                break;
            case 'Y':
                gDynamicDisp=true;
                break;
            case 'v':
                rvVersion();
                rvQueryOpenCLInfo();
                return 0;
            default:
                cout << "Error : can't handle '" << cmdstring << "', code='" << c << " ***" << endl;

            /* fall through */
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
    dfs_parameter.mode = rvDFSMode::RV_DFS_SPEED;
    dfs_parameter.filterHeight = 8;
    dfs_parameter.filterWidth = 16;
    dfs_parameter.disparity.minDisparity = gMinDisparity;
    dfs_parameter.disparity.numDisparityLevels = gLevelDisparity;
    dfs_parameter.doRectification = gDoRectification;

    rvDFSMode dfsMode = rvDFSMode::RV_DFS_SPEED;
    if (gRunningMode == 0)
    {
        dfsMode = rvDFSMode::RV_DFS_CVP;
    }
    else if (gRunningMode == 1)
    {
        dfsMode = rvDFSMode::RV_DFS_COVERAGE;
    }
    else if (gRunningMode == 2)
    {
        dfsMode = rvDFSMode::RV_DFS_SPEED;
    }
    else if (gRunningMode == 3)
    {
        dfsMode = rvDFSMode::RV_DFS_ACCURACY;
    }

    rvDFS* dfs_handle = rvDFS_Initialize(gNumCols, gNumRows, gStride, dfs_parameter, stereo_parameter);
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

std::shared_ptr<rv_dfs::DFSBase> initDFS(int width, int height, int stride, const rvStereoCamera& stereo_parameter)
{
    //prepare DFS parameters
    rvDFSParameter dfs_parameter;
    dfs_parameter.doRectification = gDoRectification;
    dfs_parameter.useDisp = !gUseDepth;
    if(gUseDepth)
    {
        dfs_parameter.depthRange.minDepth = gMinDisparity;
        dfs_parameter.depthRange.maxDepth = gMinDisparity+gLevelDisparity;
    }
    else
    {
        dfs_parameter.disparity.minDisparity = round(gMinDisparity);
        dfs_parameter.disparity.numDisparityLevels = round(gLevelDisparity);
    }

    //create DFS instance with the user specified mode
    dfs_parameter.mode = rvDFSMode::RV_DFS_SPEED;
    if (gRunningMode == 0)
    {
        dfs_parameter.mode = rvDFSMode::RV_DFS_CVP;
    }
    else if (gRunningMode == 1)
        dfs_parameter.mode = rvDFSMode::RV_DFS_COVERAGE;
    else if (gRunningMode == 2)
    {
        dfs_parameter.mode = rvDFSMode::RV_DFS_SPEED;
    }
    else if (gRunningMode == 3)
    {
        dfs_parameter.mode = rvDFSMode::RV_DFS_ACCURACY;
    }
    else if (gRunningMode == 4)
    {
        dfs_parameter.mode = rvDFSMode::RV_DFS_NORMAL;
    }

    //validate ROI settings
    if(gRoiHeight<0 || gRoiWidth<0 || gRoiStartX+gRoiWidth>gNumCols 
        || gRoiStartY+gRoiHeight>gNumRows || gRoiStartX<0 || gRoiStartY<0)
    {
        RV_ERR("ROI setting is wrong");
        return nullptr;
    }
    
    if( (gRunningMode != 2) && (gRunningMode != 4) && (gRoiWidth > 0) )
    {
        RV_INFO("ROI only supports speed and normal mode, %d mode will continue to work on the whole image in other modes",gRunningMode);
    }
    
    std::shared_ptr<rv_dfs::DFSBase> dfsBase = rv_dfs::CreateDFSbase(dfs_parameter.mode);
    if (dfsBase == nullptr)
        return nullptr;

    //initialize the DFS instance with given parameters
    if (dfsBase->initialize(width, height, stride, dfs_parameter, stereo_parameter) != true)
    {
        RV_ERR("dfs failed to initialize");
        return nullptr;
    }

    return dfsBase;
}

void saveOutput(const std::string& fullFolder, const cv::Mat& disparityMap, const cv::Mat& depthMap, const PointCloudType& pcl, const PointCloudColorType& pclColor, cv::Mat& rectImg)
{
    if(gOutputFormat == 0)
    {
        cv::imwrite(fullFolder + "/disparityOri.png", disparityMap);
        dfs_test_tool::saveColorizedDisparity(disparityMap, fullFolder + "/disparity.png");
    }
    else if (gOutputFormat == 1)
    {
        dfs_test_tool::saveDepthImage(depthMap, fullFolder + "/depth.png");
        dfs_test_tool::saveColorizedDepthImage(depthMap, fullFolder + "/depthColor.png");
    }
    else if (gOutputFormat == 2)
    {
        dfs_test_tool::writePLYPointCloud(fullFolder + "/point_cloud.ply", pcl, disparityMap.cols, disparityMap.rows);
    }
    else if (gOutputFormat == 3)
    {
        dfs_test_tool::writePLYPointCloudColor(fullFolder + "/point_cloud_color.ply", pclColor, disparityMap.cols, disparityMap.rows);
    }
    else if(gOutputFormat == 4) // save pc, pcc, disparity, depth
    {
        dfs_test_tool::writePLYPointCloud(fullFolder + "/point_cloud.ply", pcl, disparityMap.cols, disparityMap.rows);
        dfs_test_tool::writePLYPointCloudColor(fullFolder + "/point_cloud_color.ply", pclColor, disparityMap.cols, disparityMap.rows);
        dfs_test_tool::saveColorizedDisparity(disparityMap, fullFolder + "/disparity.png");
        dfs_test_tool::saveColorizedDepthImage(depthMap, fullFolder + "/depthColor.png");
    }
    else if (gOutputFormat == 5 || gOutputFormat == 6) // depth, pc
    {
        dfs_test_tool::saveDepthImage(depthMap, fullFolder + "/depth.png");
        dfs_test_tool::saveColorizedDepthImage(depthMap, fullFolder + "/depthColor.png");
        dfs_test_tool::writePLYPointCloud(fullFolder + "/point_cloud.ply", pcl, disparityMap.cols, disparityMap.rows);
    }
    else if (gOutputFormat == 7) // depth, pc, rect
    {
        dfs_test_tool::saveDepthImage(depthMap, fullFolder + "/depth.png");
        dfs_test_tool::saveColorizedDepthImage(depthMap, fullFolder + "/depthColor.png");
        dfs_test_tool::writePLYPointCloud(fullFolder + "/point_cloud.ply", pcl, disparityMap.cols, disparityMap.rows);
        cv::imwrite(fullFolder + "/rectifiedLeftInput.png", rectImg);
    }
}

void testCases(std::shared_ptr<rv_dfs::DFSBase> dfsBase, uint8_t* pLImg, uint8_t* pRImg, cv::Mat& disparityMap, cv::Mat& depthMap, PointCloudType& pcl, PointCloudColorType& pclColor, cv::Mat& rectImg)
{
    if (gOutputFormat == 0)
    {
        //calculate disparity map
        dfsBase->calculateDisparity(pLImg, pRImg, disparityMap.ptr<float>());
    }
    else if (gOutputFormat == 1)
    {
        // calculate depth image
        dfsBase->calculateDepth(pLImg, pRImg, depthMap.ptr<float>());
    }
    else if (gOutputFormat == 2)
    {
        // calculate point cloud
        dfsBase->calculatePointCloud(pLImg, pRImg, &pcl);
    }
    else if( gOutputFormat == 3)
    {
        // calculate point cloud data fusion with left image
        dfsBase->calculatePointCloudColor(pLImg, pRImg, &pclColor);
    }
    else if( gOutputFormat == 4)
    {
        // get disparity/depth/point cloud/point cloud fusion with left image data in one API
        dfsBase->calculateDispDepthPointCloud(pLImg, pRImg, disparityMap.ptr<float>(), depthMap.ptr<float>(), &pcl, &pclColor);
    }
    else if( gOutputFormat == 5)
    {
        // get depth and point cloud in one API, set unwanted disparity and point cloud output data as nullptr
        dfsBase->calculateDispDepthPointCloud(pLImg, pRImg, nullptr, depthMap.ptr<float>(), &pcl, nullptr);
    }
    else if( gOutputFormat == 6)
    {
        // get depth map and point cloud
        dfsBase->calculateDfsAllInfo(pLImg, pRImg, nullptr, depthMap.ptr<float>(), &pcl, nullptr, nullptr, nullptr);
    }
    else if( gOutputFormat == 7)
    {
        // get depth map, point cloud and rectified left image
        dfsBase->calculateDfsAllInfo(pLImg, pRImg, nullptr, depthMap.ptr<float>(), &pcl, nullptr, rectImg.ptr<uint8_t>(), nullptr);
    }
}


void runRegularTest(std::shared_ptr<rv_dfs::DFSBase> dfsBase, cv::Mat& leftImage, cv::Mat& rightImage, cv::Mat& disparityMap, cv::Mat& depthMap, PointCloudType& pcl, PointCloudColorType& pclColor, cv::Mat& rectImg)
{
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
    
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < gNumLoops; i++)
    {
        testCases(dfsBase,pLImg,pRImg,disparityMap,depthMap,pcl,pclColor, rectImg);
    }
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    cout << "[INFO] Regular test elapsed time (ms) is:   " << elapsed.count() * 1000 / gNumLoops << std::endl;
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


void runFixedFPSTest(std::shared_ptr<rv_dfs::DFSBase> dfsBase, cv::Mat& leftImage, cv::Mat& rightImage, cv::Mat& disparityMap, cv::Mat& depthMap, PointCloudType& pcl, PointCloudColorType& pclColor, cv::Mat& rectImg)
{
    assert(gFPS > 0);
    assert(gNumRows > 0 && gNumCols > 0);
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

    auto start = std::chrono::high_resolution_clock::now();
    auto durFrame = std::chrono::duration<double>(1.0 / gFPS);
    auto thres = std::chrono::duration<double>(0.000001);
    auto tsPrev = start;
    for (int i = 0; i < gNumLoops; i++)
    {
        testCases(dfsBase,pLImg,pRImg,disparityMap,depthMap,pcl,pclColor, rectImg);

        auto ts = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dur = durFrame - (ts - tsPrev);
        if (dur > thres)
        {
            std::this_thread::sleep_for(dur);
        }
        tsPrev = std::chrono::high_resolution_clock::now();
    }
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    cout << "[INFO] Elapsed time (ms) is:   " << elapsed.count() * 1000 / gNumLoops << std::endl;
}


void runDynamicRangeTest(std::shared_ptr<rv_dfs::DFSBase> dfsBase, cv::Mat& leftImage, cv::Mat& rightImage, cv::Mat& disparityMap, cv::Mat& depthMap, PointCloudType& pcl, PointCloudColorType& pclColor, cv::Mat& rectImg)
{
    if (gStereoConfigFile.empty() && gDoRectification)
        return; //must provide calibration parameters if want DFS to do rectification

    assert(gNumRows > 0 && gNumCols > 0);
    
    rvDFSDisparity dRange1, dRange2;        //For dynamic disparity range settings
    dRange1.minDisparity = gMinDisparity;
    dRange1.numDisparityLevels = gLevelDisparity / 2;
    dRange2.minDisparity = gMinDisparity + dRange1.numDisparityLevels;
    dRange2.numDisparityLevels = gLevelDisparity - gLevelDisparity / 2;

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

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < gNumLoops; i++)
    {
        if (gOutputFormat == 0)
        {
            if (i % 2 == 0)
                dfsBase->calculateDisparity(pLImg, pRImg, disparityMap.ptr<float>(), &dRange1);
            else
                dfsBase->calculateDisparity(pLImg, pRImg, disparityMap.ptr<float>(), &dRange2);
        }
        else if (gOutputFormat == 1)
        {
            if (i % 2 == 0)
                dfsBase->calculateDepth(pLImg, pRImg, disparityMap.ptr<float>(), &dRange1);
            else
                dfsBase->calculateDepth(pLImg, pRImg, disparityMap.ptr<float>(), &dRange2);
        }
        else if (gOutputFormat == 2)
        {
            if (i % 2 == 0)
                dfsBase->calculatePointCloud(pLImg, pRImg, &pcl, &dRange1);
            else
                dfsBase->calculatePointCloud(pLImg, pRImg, &pcl, &dRange2);
        }
        else if (gOutputFormat == 3)
        {
            if (i % 2 == 0)
                dfsBase->calculatePointCloudColor(pLImg, pRImg, &pclColor, &dRange1);
            else
                dfsBase->calculatePointCloudColor(pLImg, pRImg, &pclColor, &dRange2);
        }
        else if(gOutputFormat == 4) // disparity, depth, pc, pcc
        {
            if (i % 2 == 0)
                dfsBase->calculateDispDepthPointCloud(pLImg, pRImg, disparityMap.ptr<float>(), depthMap.ptr<float>(), &pcl, &pclColor, &dRange1);
            else
                dfsBase->calculateDispDepthPointCloud(pLImg, pRImg, disparityMap.ptr<float>(), depthMap.ptr<float>(), &pcl, &pclColor, &dRange2);
        }
    }
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    cout << "[INFO] Elapsed time (ms) is:   " << elapsed.count() * 1000 / gNumLoops << std::endl;
}


void TestCppUserCoordinate(std::shared_ptr<rv_dfs::DFSBase> dfsBase, cv::Mat& leftImage, cv::Mat& rightImage, PointCloudType& pcl, PointCloudColorType& pclColor, std::string& fullFolder)
{
    int m =3;
    std::vector<std::vector<float>> u2c_vec(m, std::vector<float>(12, 0.0));
    u2c_vec[0][0] = 1, u2c_vec[0][5] = 1, u2c_vec[0][10] = 1;
    u2c_vec[0][3] = 0, u2c_vec[0][7] = 0, u2c_vec[0][11] =0;
    
    u2c_vec[1][0] = 1, u2c_vec[1][5] = 1, u2c_vec[1][10] = 1;
    u2c_vec[1][3] = 10, u2c_vec[1][7] = 5, u2c_vec[1][11] =2;
    
    u2c_vec[2][0] = cos(0.524), u2c_vec[2][1] = -sin(0.524); // 30 degree = 0.524 rad
    u2c_vec[2][4] = sin(0.524), u2c_vec[2][5] = cos(0.524);
    u2c_vec[2][10] = 1;
    u2c_vec[2][3] = -10, u2c_vec[2][7] = 0, u2c_vec[2][11] =0;

    float fm[3] = {10, 5, 2}; // same to u2c_vec[1]

    int pixel_num = gNumRows* gNumCols;

    uint8_t* imgl;
    uint8_t* imgr;
    imgl = leftImage.ptr<uint8_t>();
    imgr = rightImage.ptr<uint8_t>();

    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point stop;
    std::chrono::duration<double> elapsed;
    // cal pc
    for(int i=0; i<m; i++)
    {
        dfsBase->calculatePointCloudInUserCoordinate(imgl, imgr, &pcl, u2c_vec[i].data());
        dfs_test_tool::writePLYPointCloud(fullFolder + "/pc_u2c_"+ to_string(i) + ".ply", pcl, gNumCols, gNumRows);
    }
    dfsBase->calculatePointCloudAddOffset3(imgl, imgr, &pcl, fm);
    dfs_test_tool::writePLYPointCloud(fullFolder + "/pc_u2c_offset" + ".ply", pcl, gNumCols, gNumRows);

    // cal pcc
    for(int i=0; i<m; i++)
    {
        dfsBase->calculatePointCloudColorInUserCoordinate(imgl, imgr, &pclColor, u2c_vec[i].data());
        dfs_test_tool::writePLYPointCloudColor(fullFolder + "/pcc_u2c_"+ to_string(i) + ".ply", pclColor, gNumCols, gNumRows);
    }
    dfsBase->calculatePointCloudColorAddOffset3(imgl, imgr, &pclColor, fm);
    dfs_test_tool::writePLYPointCloudColor(fullFolder + "/pcc_u2c_offset" + ".ply", pclColor, gNumCols, gNumRows);

    PointCloudType pc2;
    PointCloudColorType pcc2;
    pc2.reserve(pixel_num);
    pcc2.reserve(pixel_num);

    dfsBase->transformPointCloud(&pcl, &pc2, u2c_vec[2].data());
    dfsBase->transformPointCloud(&pc2, &pc2, u2c_vec[2].data()); // inplace

    // timing
    int loops=gNumLoops;

    start = std::chrono::high_resolution_clock::now();
    dfsBase->calculatePointCloudInUserCoordinate(imgl, imgr, &pcl, u2c_vec[2].data()); 
    stop = std::chrono::high_resolution_clock::now();
    elapsed = stop - start;
    cout << "[INFO] Time(ms) of cal pc with matmul is: " << elapsed.count() * 1000 / loops << std::endl;

    start = std::chrono::high_resolution_clock::now();
    dfsBase->calculatePointCloudAddOffset3(imgl, imgr, &pcl, fm);
    stop = std::chrono::high_resolution_clock::now();
    elapsed = stop - start;
    cout << "[INFO] Time(ms) of cal pc with add is: " << elapsed.count() * 1000 / loops << std::endl;

    start = std::chrono::high_resolution_clock::now();
    dfsBase->calculatePointCloudColorInUserCoordinate(imgl, imgr, &pclColor, u2c_vec[2].data());
    stop = std::chrono::high_resolution_clock::now();
    elapsed = stop - start;
    cout << "[INFO] Time(ms) of cal pc with matmul is: " << elapsed.count() * 1000 / loops << std::endl;

    start = std::chrono::high_resolution_clock::now();
    dfsBase->calculatePointCloudColorAddOffset3(imgl, imgr, &pclColor, fm);
    stop = std::chrono::high_resolution_clock::now();
    elapsed = stop - start;
    cout << "[INFO] Time(ms) of cal pcc with add is: " << elapsed.count() * 1000 / loops << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for(int i=0; i<loops; i++)
    {
        dfsBase->transformPointCloud(&pcl, &pc2, u2c_vec[2].data());
    }
    stop = std::chrono::high_resolution_clock::now();
    elapsed = stop - start;
    cout << "[INFO] Time(ms) of transformPointCloud is: " << elapsed.count() * 1000 / loops << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for(int i=0; i<loops; i++)
    {
        dfsBase->transformPointCloudColor(&pclColor, &pcc2, u2c_vec[2].data());
    }
    stop = std::chrono::high_resolution_clock::now();
    elapsed = stop - start;
    cout << "[INFO] Time(ms) of transformPointCloudColor is: " << elapsed.count() * 1000 / loops << std::endl;

    dfs_test_tool::writePLYPointCloud(fullFolder + "/pc_trans.ply", pc2, gNumCols, gNumRows);
    dfs_test_tool::writePLYPointCloudColor(fullFolder + "/pcc_trans.ply", pcc2, gNumCols, gNumRows);
}

void runTestRoiCpp(std::shared_ptr<rv_dfs::DFSBase> dfsBase, cv::Mat& leftImage, cv::Mat& rightImage, cv::Mat& disparityMap, cv::Mat& depthMap, PointCloudType& pcl, PointCloudColorType& pclColor, cv::Mat& rectImg)
{
    uint8_t* pLImg, * pRImg;
    pLImg = leftImage.ptr<uint8_t>();
    if (!gLeftImage.empty() && gRightImage.empty())
    {
        pRImg = nullptr;
    }
    else if(gLeftImage.empty() && !gRightImage.empty())
    {
        //images are top down 
        pLImg = rightImage.ptr<uint8_t>();
        pRImg = pLImg + gNumRows * gStride;
    }
    else
    {
        pRImg = rightImage.ptr<uint8_t>();
    }
    assert(gNumRows > 0 && gNumCols > 0 && pRImg != NULL);

    //set ROI range
    dfsBase->setROI(gRoiStartX,gRoiStartY,gRoiWidth,gRoiHeight);

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < gNumLoops; i++)
    {
        testCases(dfsBase,pLImg,pRImg,disparityMap,depthMap,pcl,pclColor, rectImg);
    }
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    cout << "[INFO] Roi Elapsed time (ms) is:   " << elapsed.count() * 1000 / gNumLoops << std::endl;
}


bool checkTargetWorkingSize(int imgw, int imgh, int neww, int newh)
{
    if( neww==0 || newh==0)
    {
        RV_ERR("new w and h must be not zero!");
        return false;
    }
    if( neww!= imgw || newh != imgh)
    {
        if (imgw < neww)
        {
            RV_ERR("New size must be not bigger than the raw input image size! imgw %d, neww %d", imgw, neww);
            return false;
        }
        if (fabs(1.0 *  imgw/ neww - 1.0 * imgh / newh) > 0.05)
        {
            RV_ERR("the ratio of resizing h and w must be equal! The info imgw %d neww %d imgh %d neww %d", imgw, neww, imgh, newh);
            return false;
        }
    }
    return true;
}

int main(int argc, const char* argv[])
{
    if (parseCommandLine(argc, argv) <= 0)
    {
        printHelp(argv[0]);
        return -1;
    }

    // fs::path abs_path = fs::system_complete( gLeftImage);
    // gLeftImage = abs_path.string();
    // printf("left img path: %s\n", gLeftImage.c_str());

    rvStereoCamera stereo_parameter;
    if (!gStereoConfigFile.empty())
    {
        //load parameter files
        stereo_parameter = dfs_test_tool::importStereoCalData(gStereoConfigFile);
        if (stereo_parameter.camera[0].pixelWidth  == 0)
        {
            RV_ERR("Invalid camara configuration!");
            return -1;
        }
    }

    if (gStereoConfigFile.empty() && (gUseDepth || gDoRectification))
    {
        RV_ERR("Stereo camera parameters are required");
        return false; //must provide calibration parameters if rectification or depth range are inputed
    }

    //// (examples)
    //// ./rv_dfs_test -l im0_vga.png -r im1_vga.png -d 1 -D 128 -m 2 -o 0
    //// ./rv_dfs_test -l side_by_side_1280_400_550.yuv -w 640 -h 400 -s 1536 -c fisheye_1280x400.yml -u 1 -d 1 -D 64 -m 0 -o 0
    //// ./rv_dfs_test -l side.yuv -w 1280 -h 800 -s 1536 -W 640 -H 400 -c fisheye_1280.yml .....
    //// ./rv_dfs_test -r top_down_1280_1600_786.yuv -w 1280 -h 800 -s 1536 -c fisheye_1280x800_topdown.yml -u 1 -d 1 -D 128 -m 2 -o 0
    //// ./rv_dfs_test -r case1/image_S_1.png -c case1/stereo_cal_1280x800.yml -w 1280 -h 800 -s 1280 -m 1 -d 1 -D 96 -W 640 -H 400 -u 1 -o 0
    ////
    //// 1) input image setting.
    ////       only left image, means side-by-side, with only -l.
    ////       only right image, means top-by-down, with only -r.
    ////       normal left and right images, with -l and -r parameters.
    ////    yuv input must set (-w,-h,-s) 
    ////    one input image of png/bmp format must set (-w,-h,-s)
    ////    two input images of png/bmp format can work without set (-w,-h,-s). Because OpenCV Mat can get image size automatically.
    ////    
    //// 2) raw setting. (gNumCols, gNumRows) is a true single left image size. And (gStride, gNumRows) need to be passed to dfs_base for initialization.
    ////    They are global init setting, and then changed by cmd setting(-w, -h, -s). image format YUV must need these cmd setting(-w, -h, -s) to get exact image data.
    ////
    //// 3) resize setting. One choice is by (gResizeCols, gResizeRows) which is got by cmd setting (-W -H). 
    ////    The other choice is changing image size in stereo config file (cw,ch for short). The (-W -H) is with higher priority.
    ////
    //// 4) adapt stereo file param and adap image size. 
    ////    only left image, size (gStride, gNumRows)  -> (resizeStride, gResizeRows) or (cs, ch)
    ////
    ////    only right image, size (gStride, 2* gNumRows)  -> (resizeStride, 2* gResizeRows) or (cs, 2*ch)
    ////    left and right image, both sizes are (gStride, gNumRows). Need to change by ratio.
    ////
    //// 5) finally update (gNumCols, gNumRows, gStride) to init dfs_base. It's the key info.
    int ch =0;
    int cw =0;
    int inputHeight = gNumRows;
    cv::Mat leftImage, rightImage;
    if (gRightImage.empty() && !gLeftImage.empty())
    {
        dfs_test_tool::readImage(gLeftImage.c_str(), leftImage, &gNumCols, &gNumRows, &gStride, true); // yuv need true image_width*2
        if(gStride>gNumCols*2)
        {
            gStride = 2* gNumCols;  // double width
            leftImage = leftImage(cv::Rect(0, 0, gStride, gNumRows)).clone();
        }
    }
    else if(gLeftImage.empty() && !gRightImage.empty())
    {
        inputHeight = gNumRows*2;
        dfs_test_tool::readImage(gRightImage.c_str(), rightImage, &gNumCols, &inputHeight, &gStride, false);
        gNumRows = inputHeight/2;
        if(gStride>gNumCols)
        {
            rightImage = rightImage(cv::Rect(0, 0, gNumCols, 2*gNumRows)).clone();
            gStride = gNumCols;
        }
    }
    else
    {
        dfs_test_tool::readImage(gLeftImage.c_str(), leftImage, &gNumCols, &gNumRows, &gStride, false);
        dfs_test_tool::readImage(gRightImage.c_str(), rightImage, &gNumCols, &gNumRows, &gStride, false);
        if(gStride>gNumCols)
        {
            leftImage = leftImage(cv::Rect(0, 0, gNumCols, gNumRows)).clone();
            rightImage = rightImage(cv::Rect(0, 0, gNumCols, gNumRows)).clone();
        }
    }

    /// hw setting priority: cmd setting > stereo_file > img raw hw > global init value.
    if( gHwCmdSet )
    {
        if (!checkTargetWorkingSize(gNumCols, gNumRows, gResizeCols, gResizeRows))
            return -1;
        // adapt stereo param.
        if( !gStereoConfigFile.empty())
        {
            cw = stereo_parameter.camera[0].pixelWidth;
            ch = stereo_parameter.camera[0].pixelHeight;
            if (!checkTargetWorkingSize(gNumCols, gNumRows, cw, ch))
                return -1;
            float facW = float(gResizeCols) / float(cw);
            float facH = float(gResizeRows) / float(ch);
            dfs_test_tool::scaleIntrinsics(stereo_parameter, facW, facH); // update fx, fy, px, py, sh, sw
        }

        // adapt img hw.
        if( gRightImage.empty() && !gLeftImage.empty())
            cv::resize(leftImage, leftImage, cv::Size(2*gResizeCols, gResizeRows), 0, 0, cv::INTER_CUBIC);
        else if( gLeftImage.empty() && !gRightImage.empty())
            cv::resize(rightImage, rightImage, cv::Size(gResizeCols, 2*gResizeRows), 0, 0, cv::INTER_CUBIC);
        else
        {
            cv::resize(leftImage, leftImage, cv::Size(gResizeCols, gResizeRows), 0, 0, cv::INTER_CUBIC);
            cv::resize(rightImage, rightImage, cv::Size(gResizeCols, gResizeRows), 0, 0, cv::INTER_CUBIC);
        }
        gStride = int( float(gStride * gResizeCols)/ gNumCols + 0.5f);
        gNumCols = gResizeCols;  // update gNumCols to init dfs_base
        gNumRows = gResizeRows;
    }
    else if( !gStereoConfigFile.empty())
    {
        cw = stereo_parameter.camera[0].pixelWidth;
        ch = stereo_parameter.camera[0].pixelHeight;
        if (checkTargetWorkingSize(gNumCols, gNumRows, cw, ch) == false)
            return -1;

        // adapt img hw.
        if( gRightImage.empty() && !gLeftImage.empty())
            cv::resize(leftImage, leftImage, cv::Size(2*cw, ch), 0, 0, cv::INTER_CUBIC);
        else if( gLeftImage.empty() && !gRightImage.empty())
            cv::resize(rightImage, rightImage, cv::Size(cw, 2*ch), 0, 0, cv::INTER_CUBIC);
        else
        {
            cv::resize(leftImage, leftImage, cv::Size(cw, ch), 0, 0, cv::INTER_CUBIC);
            cv::resize(rightImage, rightImage, cv::Size(cw, ch), 0, 0, cv::INTER_CUBIC);
        }
        gStride = int( float(gStride * cw)/ gNumCols + 0.5f);
        gNumCols = cw; // update gNumCols to init dfs_base
        gNumRows = ch;
    }

    //prepare data for DFS results
    PointCloudType pcl;
    pcl.reserve(gNumCols * gNumRows);
    PointCloudColorType pclColor;
    pclColor.reserve(gNumCols * gNumRows);
    cv::Mat disparityMap = cv::Mat::zeros(gNumRows, gNumCols, CV_32FC1);
    cv::Mat depthMap = cv::Mat::zeros(gNumRows, gNumCols, CV_32FC1);
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

    std::string fullFolder = gLeftImage;
    int s = fullFolder.find_last_of("\\");
    if (s < 0)
        s = fullFolder.find_last_of("/");
    if (s <= 0)
        fullFolder = ".";
    else
        fullFolder.resize(s);

    std::shared_ptr<rv_dfs::DFSBase> dfsBase = initDFS(gNumCols,gNumRows,gStride,stereo_parameter);
    
    //once initialized, user can use this API to get new intrinsics/extrinsics of rectified image pair without running DFS
    rvStereoCamera rectified_stereo_parameter = dfsBase->getRectifiedCameraParameter();

#ifdef DFS_CPP_STYLE_INTERFACE
    if(gFPS>0)
    {
        runFixedFPSTest(dfsBase, leftImage, rightImage, disparityMap, depthMap, pcl, pclColor, rectLImg);
    }
    else if(gRoiStartX>0 || gRoiStartY>0 || gRoiWidth>0 || gRoiHeight>0)
    {
        runTestRoiCpp(dfsBase, leftImage, rightImage, disparityMap, depthMap, pcl, pclColor, rectLImg);
    }
    else if(gDynamicDisp)
    {
        runDynamicRangeTest(dfsBase, leftImage, rightImage, disparityMap, depthMap, pcl, pclColor, rectLImg);
    }
    else
    {
        runRegularTest(dfsBase, leftImage, rightImage, disparityMap, depthMap, pcl, pclColor, rectLImg);
    }
    // status &= TestCppUserCoordinate(dfsBase, leftImage, rightImage, pcl, pclColor, fullFolder);
    
    saveOutput(fullFolder, disparityMap, depthMap, pcl, pclColor, rectLImg);

    //if rectification is done by DFS, save rectified images
    if (gDoRectification) 
    {
        dfsBase->getRectImages(rectLImg.ptr<uint8_t>(), rectRImg.ptr<uint8_t>());
        cv::imwrite(fullFolder + "/rightRectifiedImage.png", rectRImg);
        cv::imwrite(fullFolder + "/leftRectifiedImage.png", rectLImg);
    }

    //finally de-initialize the DFS instance
    dfsBase->deInitialize();
#else
    RunTestC(leftImage, rightImage, stereo_parameter, fullFolder);
#endif

    RV_INFO("%s finished!",gLeftImage.c_str());

    return 0;
}
