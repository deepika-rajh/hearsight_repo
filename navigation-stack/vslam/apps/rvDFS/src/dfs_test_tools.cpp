/*****************************************************************************
@copyright
Copyright (c) 2021 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/


#include "dfs_test_tools.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>


namespace dfs_test_tool {
	typedef std::vector<std::vector<double>> PointCloudType;

	std::vector<float> transferDisparyToDepth(int width, int height, int stride, 
		const float* disparity_map, double factor, double doffs)
	{
		if (nullptr == disparity_map)
		{
			std::cout << "Input disparity_map is empty." << std::endl;
			return std::vector<float>();
		}
		std::vector<float> depth_map(width * height);
		int i = 0;
		for (int h = 0; h < height; ++h)
		{
			for (int w = 0; w < width; ++w)
			{
				if (disparity_map[h * stride + w] <= 1.0e-8) {
					depth_map[i++] = 0.0;
					continue;
				}

				depth_map[i++] = factor / (disparity_map[h * stride + w] + doffs);
			}
		}
		return depth_map;
	}


	cv::Mat transferDisparyToDepthU16(int width, int height, int stride, const float* disparity_map, double factor, double doffs)
	{
		if (nullptr == disparity_map)
		{
			std::cout << "Input disparity_map is empty." << std::endl;
			return cv::Mat();
		}

		const int rows = height;
		const int cols = width;

		cv::Mat depth_map(rows, cols, CV_16U);

		for (int h = 0; h < height; ++h)
		{
			for (int w = 0; w < width; ++w)
			{
				const int r = h;
				const int c = w;
				if (disparity_map[h * stride + w] <= 1.0e-8) {
					depth_map.at<uint16_t>(r, c) = 0;
					continue;
				}
				double d = factor / (disparity_map[h * stride + w] + doffs);
				depth_map.at<uint16_t>(r, c) = static_cast<uint16_t>(d);
			}
		}
		return depth_map;
	}


	void writePLYPointcloud(std::ostream& o_st, const PointCloudType& pointcloud)
	{
		o_st << "ply" << std::endl;
		o_st << "format ascii 1.0" << std::endl;
		o_st << "element vertex " << pointcloud.size() << std::endl;
		o_st << "property float x" << std::endl;
		o_st << "property float y" << std::endl;
		o_st << "property float z" << std::endl;
		o_st << "end_header" << std::endl;
		for (const auto& pc : pointcloud)
		{
			o_st << std::fixed << std::setprecision(2) << pc[0] << " " << pc[1] << " " << pc[2] << std::endl;
		}
	}


	void exportPLYFile(int width, int height, int stride,
		const float* disparity_map, const rvStereoConfiguration& stereo_config,
		const std::string& ply_file_path)
	{
		if (nullptr == disparity_map)
		{
			std::cout << "Input disparity_map is empty." << std::endl;
			return;
		}
		const double doffs = stereo_config.camera[1].principalPoint[0] -
			stereo_config.camera[0].principalPoint[0];
		const rvCameraConfiguration& camera_config = stereo_config.camera[0];
		const double focal_len_u = camera_config.focalLength[0];
		const double focal_len_v = camera_config.focalLength[1];
		const double cx = camera_config.principalPoint[0];
		const double cy = camera_config.principalPoint[1];
		double T = stereo_config.translation[0];

		const double factor = focal_len_u * T;
		cv::Mat depth_mat_u16 = transferDisparyToDepthU16(width,
			height, stride, disparity_map, factor, doffs);
		cv::imwrite("depthMapU16.png", depth_mat_u16);

		std::vector<float> depth_map = transferDisparyToDepth(width,
			height, stride, disparity_map, factor, doffs);
		PointCloudType point_cloud;
		for (int h = 0; h < height; ++h)
		{
			for (int w = 0; w < width; ++w)
			{
				double d = depth_map[h * width + w];
				if (std::abs(d) <= 1.0e-8) continue;
				double X = (w - cx) * d / focal_len_u;
				double Y = (h - cy) * d / focal_len_v;
				point_cloud.push_back(std::vector<double>({ X, Y, d }));
			}
		}
		std::ofstream f_st(ply_file_path);
		writePLYPointcloud(f_st, point_cloud);
	}

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
		dfs_parameter.camera[0].distortionModel = 5;
		for (int i = 0; i < 5; ++i)
			dfs_parameter.camera[0].distortion[i] = distortion_left.at<double>(0, i);

		cv::Mat camera_mat_right;
		fs["Camera_Matrix2"] >> camera_mat_right;
		dfs_parameter.camera[1].focalLength[0] = camera_mat_right.at<double>(0, 0);
		dfs_parameter.camera[1].focalLength[1] = camera_mat_right.at<double>(1, 1);
		dfs_parameter.camera[1].principalPoint[0] = camera_mat_right.at<double>(0, 2);
		dfs_parameter.camera[1].principalPoint[1] = camera_mat_right.at<double>(1, 2);

		cv::Mat distortion_right;
		fs["Distortion_Coefficients2"] >> distortion_right;
		for (int i = 0; i < 5; ++i)
			dfs_parameter.camera[1].distortion[i] = distortion_right.at<double>(0, i);

		cv::FileNode image_size_node = fs["Image_Size"];
		std::vector<int> image_size;
		image_size_node >> image_size;
		dfs_parameter.camera[0].pixelWidth = image_size[0];
		dfs_parameter.camera[0].pixelHeight = image_size[1];
		dfs_parameter.camera[1].pixelWidth = image_size[0];
		dfs_parameter.camera[1].pixelHeight = image_size[1];

		cv::Mat R;
		fs["R"] >> R;
		cv::Mat T;
		fs["T"] >> T;

		cv::Vec3d rot_rodrigues;
		cv::Rodrigues(R, rot_rodrigues);
		for (int i = 0; i < 3; ++i)
		{
			dfs_parameter.translation[i] = T.at<double>(i, 0);
			dfs_parameter.rotation[i] = rot_rodrigues[i];
		}

		return dfs_parameter;
	}

	rvStereoConfiguration importStereoCalData(const std::string& file)
	{
		return ocvImportStereoCalData(file);
	}
}
