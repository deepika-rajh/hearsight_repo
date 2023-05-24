/*****************************************************************************
@copyright
Copyright (c) 2022-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#include "dfs_test_tools.h"
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include "rvLog.h"
#include <chrono>
#include <opencv2/calib3d.hpp>
#include <typeinfo>
#ifdef WIN32
#define strncasecmp(x, y, z) _strnicmp(x, y, z)
#define strcasecmp stricmp
#endif

namespace dfs_test_tool
{
    void writePLYPointCloud(const std::string &ply_file_path, const PointCloudType &pointCloud, size_t width, size_t height)
    {
        std::ofstream o_st(ply_file_path);
        o_st << "ply" << std::endl;
        o_st << "format ascii 1.0" << std::endl;
        o_st << "element vertex " << pointCloud.size() << std::endl;
        o_st << "property float x" << std::endl;
        o_st << "property float y" << std::endl;
        o_st << "property float z" << std::endl;
        o_st << "end_header" << std::endl;
        for (const auto &pc : pointCloud)
        {
            o_st << std::fixed << std::setprecision(2) << pc[0] << " " << pc[1] << " " << pc[2] << std::endl;
        }
    }

    void writePLYPointCloudColor(const std::string &ply_file_path, const PointCloudColorType &pointCloud, size_t width, size_t height)
    {
        std::ofstream o_st(ply_file_path);
        o_st << "ply" << std::endl;
        o_st << "format ascii 1.0" << std::endl;
        o_st << "element vertex " << pointCloud.size() << std::endl;
        o_st << "property float x" << std::endl;
        o_st << "property float y" << std::endl;
        o_st << "property float z" << std::endl;
        o_st << "property uchar red" << std::endl;
        o_st << "property uchar green" << std::endl;
        o_st << "property uchar blue" << std::endl;
        o_st << "end_header" << std::endl;
        for (const auto &pc : pointCloud)
        {
            unsigned int gray = pc[3] > 255.0 ? 255 : static_cast<unsigned>(pc[3]);
            o_st << std::fixed << std::setprecision(2) << pc[0] << " " << pc[1] << " " << pc[2]
                 << " " << gray << " " << gray << " " << gray << std::endl;
        }
    }

	//Translation should be millimeter
	rvStereoCamera ocvImportStereoCalData(const std::string& file)
	{
		rvStereoCamera dfs_parameter;
		dfs_parameter.camera[0].pixelWidth = 0;
		dfs_parameter.camera[0].pixelHeight = 0;
		//load parameter files
		cv::FileStorage fs(file, cv::FileStorage::READ);
		if (!fs.isOpened())
		{
			return dfs_parameter;
		}

        cv::Mat camera_mat_left;
        fs["Camera_Matrix1"] >> camera_mat_left;
        dfs_parameter.camera[0].focalLength[0] = camera_mat_left.at<double>(0, 0);
        dfs_parameter.camera[0].focalLength[1] = camera_mat_left.at<double>(1, 1);
        dfs_parameter.camera[0].principalPoint[0] = camera_mat_left.at<double>(0, 2);
        dfs_parameter.camera[0].principalPoint[1] = camera_mat_left.at<double>(1, 2);
        cv::Mat distortion_left;
        fs["Distortion_Coefficients1"] >> distortion_left;

        cv::Mat camera_mat_right;
        fs["Camera_Matrix2"] >> camera_mat_right;
        dfs_parameter.camera[1].focalLength[0] = camera_mat_right.at<double>(0, 0);
        dfs_parameter.camera[1].focalLength[1] = camera_mat_right.at<double>(1, 1);
        dfs_parameter.camera[1].principalPoint[0] = camera_mat_right.at<double>(0, 2);
        dfs_parameter.camera[1].principalPoint[1] = camera_mat_right.at<double>(1, 2);

        cv::Mat distortion_right;
        fs["Distortion_Coefficients2"] >> distortion_right;

        cv::FileNode image_size_node = fs["Image_Size"];
        std::vector<int> image_size;
        image_size_node >> image_size;
        dfs_parameter.camera[0].pixelWidth = image_size[0];
        dfs_parameter.camera[0].pixelHeight = image_size[1];
        dfs_parameter.camera[1].pixelWidth = image_size[0];
        dfs_parameter.camera[1].pixelHeight = image_size[1];

        std::string distortionModel;
        fs["distortion_model"] >> distortionModel;

        if (strcasecmp(distortionModel.c_str(), "equidistant") == 0 ||
            strncasecmp(distortionModel.c_str(), "fisheye", 7) == 0)
        {
            dfs_parameter.camera[0].distortionModel = FisheyeModel4;
            dfs_parameter.camera[1].distortionModel = FisheyeModel4;
            for (int i = 0; i < 4; ++i)
            {
                if (distortion_left.rows > 1)
                    dfs_parameter.camera[0].distortion[i] = distortion_left.at<double>(i, 0);
                else
                    dfs_parameter.camera[0].distortion[i] = distortion_left.at<double>(0, i);
                if (distortion_right.rows > 1)
                    dfs_parameter.camera[1].distortion[i] = distortion_right.at<double>(i, 0);
                else
                    dfs_parameter.camera[1].distortion[i] = distortion_right.at<double>(0, i);
            }
            for (int i = 4; i < 8; ++i)
            {
                dfs_parameter.camera[0].distortion[i] = 0.0;
                dfs_parameter.camera[1].distortion[i] = 0.0;
            }
        }
        else if (strcasecmp(distortionModel.c_str(), "rational_polynomial") == 0)
        {
            dfs_parameter.camera[0].distortionModel = RationalModel8;
            dfs_parameter.camera[1].distortionModel = RationalModel8;
            for (int i = 0; i < 8; ++i)
            {
                if (distortion_left.rows > 1)
                    dfs_parameter.camera[0].distortion[i] = distortion_left.at<double>(i, 0);
                else
                    dfs_parameter.camera[0].distortion[i] = distortion_left.at<double>(0, i);
                if (distortion_right.rows > 1)
                    dfs_parameter.camera[1].distortion[i] = distortion_right.at<double>(i, 0);
                else
                    dfs_parameter.camera[1].distortion[i] = distortion_right.at<double>(0, i);
            }
        }
        else //"Plumb_bob" or default
        {
            dfs_parameter.camera[0].distortionModel = Polynomial5;
            dfs_parameter.camera[1].distortionModel = Polynomial5;
            for (int i = 0; i < 5; ++i)
            {
                if (distortion_left.rows > 1)
                    dfs_parameter.camera[0].distortion[i] = distortion_left.at<double>(i, 0);
                else
                    dfs_parameter.camera[0].distortion[i] = distortion_left.at<double>(0, i);
                if (distortion_right.rows > 1)
                    dfs_parameter.camera[1].distortion[i] = distortion_right.at<double>(i, 0);
                else
                    dfs_parameter.camera[1].distortion[i] = distortion_right.at<double>(0, i);
            }
            for (int i = 5; i < 8; ++i)
            {
                dfs_parameter.camera[0].distortion[i] = 0.0;
                dfs_parameter.camera[1].distortion[i] = 0.0;
            }
        }

        cv::Mat R;
        fs["R"] >> R;
        // Translation should be millimeter
        cv::Mat T;
        fs["T"] >> T;
        cv::Vec3d rot_rodrigues;
        if (R.total() > 3)
            cv::Rodrigues(R, rot_rodrigues);
        for (int i = 0; i < 3; ++i)
        {
            dfs_parameter.translation[i] = T.at<double>(i, 0);
            if (R.total() > 3)
                dfs_parameter.rotation[i] = rot_rodrigues[i];
            else
                dfs_parameter.rotation[i] = R.at<double>(i, 0);
        }

        return dfs_parameter;
    }

    //Translation should be millimeter
    rvStereoCamera importStereoCalData(const std::string& file)
    {
        return ocvImportStereoCalData(file);
    }

    void writeIntrinsic(const rvStereoCamera& param, const float factorW, const float factorH, const std::string& file)
    {
        cv::FileStorage fs(file, cv::FileStorage::WRITE);
        if( !fs.isOpened())
        {
            std::cout << "Error: can't open file " << file << std::endl;
            return ;
        }
        cv::Mat camera_left = cv::Mat::zeros(3, 3, CV_64FC1);
        cv::Mat camera_right = cv::Mat::zeros(3, 3, CV_64FC1);

        camera_left.at<double>(0, 0) = param.camera[0].focalLength[0] * factorW;
        camera_left.at<double>(1, 1) = param.camera[0].focalLength[1] * factorH;
        camera_left.at<double>(0, 2) = param.camera[0].principalPoint[0] * factorW;
        camera_left.at<double>(1, 2) = param.camera[0].principalPoint[1] * factorH;
        camera_left.at<double>(2, 2) = 1.0;

        camera_right.at<double>(0, 0) = param.camera[1].focalLength[0] * factorW;
        camera_right.at<double>(1, 1) = param.camera[1].focalLength[1] * factorH;
        camera_right.at<double>(0, 2) = param.camera[1].principalPoint[0] * factorW;
        camera_right.at<double>(1, 2) = param.camera[1].principalPoint[1] * factorH;
        camera_right.at<double>(2, 2) = 1.0;

        fs << "Camera_Matrix1" << camera_left << "Camera_Matrix2" << camera_right;
        fs.release();
    }

	void scaleIntrinsics(rvStereoCamera& param, const float factorW, const float factorH)
	{
		param.camera[0].focalLength[0] *= factorW;
		param.camera[0].focalLength[1] *= factorH;
		param.camera[0].principalPoint[0] *= factorW;
		param.camera[0].principalPoint[1] *= factorH;

		param.camera[1].focalLength[0] *= factorW; // may different from camera 0.
		param.camera[1].focalLength[1] *= factorH;
		param.camera[1].principalPoint[0] *= factorW;
		param.camera[1].principalPoint[1] *= factorH;

		param.camera[0].pixelWidth = int(param.camera[0].pixelWidth * factorW + 0.5f);
		param.camera[0].pixelHeight = int(param.camera[0].pixelHeight * factorH + 0.5f);
		param.camera[1].pixelWidth = param.camera[0].pixelWidth;
		param.camera[1].pixelHeight = param.camera[0].pixelHeight;
	}

    void readImage(const char *imageName, cv::Mat &image, int *pWidth, int *pHeight, int *pStride, bool SBS)
    {
        std::string name(imageName);
        if (name.substr(name.length() - 3) == "yuv")
        {
            image = cv::Mat(*pHeight, *pStride, CV_8UC1);
            // Open left and right images
            FILE *inputFileL = fopen(name.c_str(), "rb");
            // Read left and right images
            fread((void *)image.data, sizeof(char), (*pStride) * (*pHeight), inputFileL);
            fclose(inputFileL);
            // width, height and stride relies on user input
        }
        else
        {
            image = cv::imread(imageName);
            if (image.channels() == 3)
            {
                cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
            }
            if (SBS)
            {
                *pWidth = image.cols / 2;
            }
            else
            {
                *pWidth = image.cols;
            }
            *pStride = image.step;
            *pHeight = image.rows;
        }
        RV_DBG("image %s size is %d x %d, stride is %d", imageName, *pWidth, *pHeight, *pStride);
    }

    void saveMap(const std::string &fullFolder, const std::string &mapName, int nameIdx, float *disparityFloat, int width, int height, int mode)
    {
        // save original disparity image
        cv::Mat disparityImageChar(height, width, CV_8UC1);
        cv::Mat disparityImageFloat(height, width, CV_32FC1);
        unsigned char *pDisparityChar = (unsigned char *)disparityImageChar.data;
        for (int ii = 0; ii < width * height; ++ii)
            pDisparityChar[ii] = static_cast<unsigned char>(round(disparityFloat[ii]));
        cv::Mat disp;
        disp = cv::Mat::zeros(height, width, CV_32FC1);
        memcpy(disp.data, disparityFloat, sizeof(float) * height * width);
        char fname[256];
        snprintf(fname, 256,"/%sOri_%d.png", mapName.c_str(), nameIdx);
        cv::imwrite(fullFolder + fname, disp);

        // save colorized disparity image
        double min;
        double max;
        cv::minMaxIdx(disparityImageChar, &min, &max);
        double scale = 255. / (max - min);
        disparityImageChar.convertTo(disparityImageChar, CV_8UC1, scale, -min * scale);
        cv::Mat falseColorsMap;
        cv::applyColorMap(disparityImageChar, falseColorsMap, cv::COLORMAP_JET);
        snprintf(fname, 256,"/%s_%d.png", mapName.c_str(), nameIdx);
        cv::imwrite(fullFolder + fname, falseColorsMap);
    }


    void saveColorizedDisparity(const cv::Mat& disparityFloat, const std::string& fullPath)
    {
        //generate fixed-point disparity image
        cv::Mat disparityImageChar(disparityFloat.size(), CV_8UC1);
        unsigned char* pDisparityChar = (unsigned char*)disparityImageChar.data;
        float* pDisparityFloat = (float*)disparityFloat.data;
        for (int ii = 0; ii < disparityFloat.cols * disparityFloat.rows; ++ii)
            pDisparityChar[ii] = static_cast<unsigned char>(round(pDisparityFloat[ii]));

        //generate and save colorized disparity image
        double min;
        double max;
        cv::minMaxIdx(disparityImageChar, &min, &max);
        double scale = 255. / (max - min);
        disparityImageChar.convertTo(disparityImageChar, CV_8UC1, scale, -min * scale);
        cv::Mat colorsMap;
        cv::applyColorMap(disparityImageChar, colorsMap, cv::COLORMAP_JET);
        cv::imwrite(fullPath, colorsMap);
    }


    void saveDepthImage(const cv::Mat& depthFloat, const std::string& fullPath)
    {
        cv::Mat depthImage(depthFloat.size(), CV_16UC1);
        unsigned short* pDepth = (unsigned short*)depthImage.data;
        const float* pFloatDisparity = depthFloat.ptr<float>();
        for (int ii = 0; ii < depthFloat.cols * depthFloat.rows; ++ii)
            pDepth[ii] = static_cast<unsigned short>(round(pFloatDisparity[ii]));
        cv::imwrite(fullPath, depthImage);
    }


    void saveColorizedDepthImage(const cv::Mat& depthFloat, const std::string& fullPath)
    {
        cv::Mat depthImage(depthFloat.size(), CV_16UC1);
        unsigned short* pDepth = (unsigned short*)depthImage.data;
        const float* pFloatDisparity = depthFloat.ptr<float>();
        for (int ii = 0; ii < depthFloat.cols * depthFloat.rows; ++ii)
            pDepth[ii] = static_cast<unsigned short>(round(pFloatDisparity[ii]));

        double min;
        double max;
        cv::minMaxIdx(depthImage, &min, &max);
        double scale = 255. / (max - min);
        depthImage.convertTo(depthImage, CV_8UC1, scale, -min * scale);
        cv::Mat colorsMap;
        cv::applyColorMap(depthImage, colorsMap, cv::COLORMAP_JET);
        cv::imwrite(fullPath, colorsMap);
    }


    void calDispWithSGBM(cv::Mat imgL, cv::Mat imgR, cv::Mat& imgDisparity8U, int dispLevel, int iterNum)
    {
        cv::Size imgSize = imgL.size();
        int numberOfDisparities = dispLevel;
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
        for (int i = 0; i < iterNum; i++)
        {
            sgbm->compute(imgL, imgR, imgDisparity16S);
        }
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;

        RV_ERR("SGBM Elapsed time: %f ms", elapsed.count() * 1000 / iterNum);
    #else
        sgbm->compute(imgL, imgR, imgDisparity16S);
    #endif
    #ifdef MIDDLEBERRY_EVAL
        cv::imwrite("disparitySGBM16S.bmp", imgDisparity16S);
        imgDisparity16S.convertTo(imgDisparity8U, CV_32F, 255 / (255 * 16.));
    #ifdef WIN32
        WriteFilePFM((float*)imgDisparity8U.data, imgDisparity8U.cols, imgDisparity8U.rows, "disp0SGBM32F.pfm");
    #else
        CShape sh(imgDisparity8U.cols, imgDisparity8U.rows, 1);
        CFloatImage fdisp;
        fdisp.ReAllocate(sh, (float*)imgDisparity8U.data, false, sh.width * sizeof(float));
        WriteFilePFM(fdisp, "disp0SGBM32F.pfm", (float)(1.0 / 255.0));
    #endif
    #endif // MIDDLEBERRY_EVAL
    }


}
