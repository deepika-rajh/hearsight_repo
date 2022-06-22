/*****************************************************************************
@copyright
Copyright (c) 2021 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/


#include "dfs_test_tools.h"

#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>


namespace dfs_test_tool {
	void writePLYPointcloud(const std::string& ply_file_path, const PointCloudType& pointCloud, size_t width, size_t height)
	{
		std::ofstream o_st(ply_file_path);
		o_st << "ply" << std::endl;
		o_st << "format ascii 1.0" << std::endl;
		o_st << "element vertex " << pointCloud.size() << std::endl;
		o_st << "property float x" << std::endl;
		o_st << "property float y" << std::endl;
		o_st << "property float z" << std::endl;
		o_st << "end_header" << std::endl;
		for (const auto& pc : pointCloud)
		{
			o_st << std::fixed << std::setprecision(2) << pc[0] << " " << pc[1] << " " << pc[2] << std::endl;
		}
	}

	void writePLYPointcloudColor(const std::string& ply_file_path, const PointCloudColorType& pointCloud, size_t width, size_t height)
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
		for (const auto& pc : pointCloud)
		{
			unsigned int gray = pc[3] > 255.0 ? 255 : static_cast<unsigned>(pc[3]);
			o_st << std::fixed << std::setprecision(2) << pc[0] << " " << pc[1] << " " << pc[2]
				<< " " << gray << " " << gray << " " << gray << std::endl;
		}
	}

	//Translation should be millimeter
	rvStereoConfiguration ocvImportStereoCalData(const std::string& file)
	{
		rvStereoConfiguration dfs_parameter;
		//load parameter files
		cv::FileStorage fs(file, cv::FileStorage::READ);
		if (!fs.isOpened()) return rvStereoConfiguration();

		cv::Mat camera_mat_left;
		fs["Camera_Matrix1"] >> camera_mat_left;
		dfs_parameter.camera[0].focalLength[0] = camera_mat_left.at<double>(0, 0);
		dfs_parameter.camera[0].focalLength[1] = camera_mat_left.at<double>(1, 1);
		dfs_parameter.camera[0].principalPoint[0] = camera_mat_left.at<double>(0, 2);
		dfs_parameter.camera[0].principalPoint[1] = camera_mat_left.at<double>(1, 2);
		cv::Mat distortion_left;
		fs["Distortion_Coefficients1"] >> distortion_left;
		// dfs_parameter.camera[0].distortionModel = 5;
		// for (int i = 0; i < 5; ++i)
		// 	dfs_parameter.camera[0].distortion[i] = distortion_left.at<double>(0, i);

		cv::Mat camera_mat_right;
		fs["Camera_Matrix2"] >> camera_mat_right;
		dfs_parameter.camera[1].focalLength[0] = camera_mat_right.at<double>(0, 0);
		dfs_parameter.camera[1].focalLength[1] = camera_mat_right.at<double>(1, 1);
		dfs_parameter.camera[1].principalPoint[0] = camera_mat_right.at<double>(0, 2);
		dfs_parameter.camera[1].principalPoint[1] = camera_mat_right.at<double>(1, 2);

		cv::Mat distortion_right;
		fs["Distortion_Coefficients2"] >> distortion_right;
		// for (int i = 0; i < 5; ++i)
		// 	dfs_parameter.camera[1].distortion[i] = distortion_right.at<double>(0, i);

		cv::FileNode image_size_node = fs["Image_Size"];
		std::vector<int> image_size;
		image_size_node >> image_size;
		dfs_parameter.camera[0].pixelWidth = image_size[0];
		dfs_parameter.camera[0].pixelHeight = image_size[1];
		dfs_parameter.camera[1].pixelWidth = image_size[0];
		dfs_parameter.camera[1].pixelHeight = image_size[1];

		std::string distortionModel;
		fs["distortion_model"] >> distortionModel;
		
		if(strcasecmp(distortionModel.c_str(), "equidistant")==0 || 
		    strncasecmp(distortionModel.c_str(), "fisheye", 7)==0)
		{
			dfs_parameter.camera[0].distortionModel = 10;
			dfs_parameter.camera[1].distortionModel = 10;
			for (int i = 0; i < 4; ++i)
			{
				dfs_parameter.camera[0].distortion[i] = distortion_left.at<double>(0, i);
				dfs_parameter.camera[1].distortion[i] = distortion_right.at<double>(0, i);
			}
			for (int i = 4; i < 8; ++i)
			{
				dfs_parameter.camera[0].distortion[i] = 0.0;
				dfs_parameter.camera[1].distortion[i] = 0.0;
			}
		}
		else if(strcasecmp(distortionModel.c_str(), "rational_polynomial") == 0)
		{
			dfs_parameter.camera[0].distortionModel = 8;
			dfs_parameter.camera[1].distortionModel = 8;
			for (int i = 0; i < 8; ++i)
			{
				dfs_parameter.camera[0].distortion[i] = distortion_left.at<double>(0, i);
				dfs_parameter.camera[1].distortion[i] = distortion_right.at<double>(0, i);
			}
		}
		else //"Plumb_bob" or default
		{
			dfs_parameter.camera[0].distortionModel = 5;
			dfs_parameter.camera[1].distortionModel = 5;
			for (int i = 0; i < 5; ++i)
			{
				dfs_parameter.camera[0].distortion[i] = distortion_left.at<double>(0, i);
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
		//Translation should be millimeter
		cv::Mat T;
		fs["T"] >> T;

		cv::Vec3d rot_rodrigues;
		// std::cout << "R size is "<< R.total();
		if(R.total() > 3)
			cv::Rodrigues(R, rot_rodrigues);
		for (int i = 0; i < 3; ++i)
		{
			dfs_parameter.translation[i] = T.at<double>(i, 0);
			if(R.total()>3)
				dfs_parameter.rotation[i] = rot_rodrigues[i];
			else
				dfs_parameter.rotation[i] = R.at<double>(i,0);
		}

		return dfs_parameter;
	}

	//Translation should be millimeter
	rvStereoConfiguration importStereoCalData(const std::string& file)
	{
		return ocvImportStereoCalData(file);
	}
}
