/*****************************************************************************
@copyright
Copyright (c) 2020-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/


#ifndef RV_DFS_BASE_H
#define RV_DFS_BASE_H

#include <string>
#include "rvDFS.h"

namespace rv_dfs
{
    /// flags for annotating disparity maps
    enum eDFSPixelState
    {
        DFS_PIXEL_ATTEMPTED = 0,
        DFS_PIXEL_CORRECTED = 1 << 1,           /// disparity corrected by post-processing
        DFS_PIXEL_FILLED = 1 << 2,              /// disparity filled by post-processing
        DFS_PIXEL_SMALLCONCOMP = 1 << 3,
        DFS_PIXEL_OK = 1 << 4,                  /// disparity at given pixel determined
        DFS_PIXEL_UNKNOWN = 1 << 5,             /// disparity at given pixel unknown
        DFS_PIXEL_OCCLUDED = 1 << 6,            /// pixel occluded        
        DFS_PIXEL_NOTTEXTURED = 1 << 7
    };

    class DFSBase
    {
    public:
        //------------------------------------------------------------------------------
        /// @detailed
        ///      Initialize
        /// @param width
        ///   Width of input images
        /// @param height
        ///   Height of input images
        /// @param stride
        ///   Stride of input images
        /// @param dfs_parameter
        ///   DFS parameters
        /// @param stereo_parameter
        ///   Intrinsic and extrinsic parameters of stereo camera
        /// @return
        ///   Returns True if success or False if failure
        //------------------------------------------------------------------------------
        virtual bool initialize(int width, int height, int stride,
                                const rvDFSParameter& dfs_parameter, const rvStereoCamera& stereo_parameter) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and get disparity map
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
        /// @param disparities
        ///   Disparity map pointer for output
        /// @param dfs_disparity
        ///   DFS disparity parameters pointer, can be null
        //------------------------------------------------------------------------------
        virtual void calculateDisparity(uint8_t* imgL, uint8_t* imgR, float* disparities, rvDFSDisparity* dfs_disparity=nullptr) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and get depth map
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
        /// @param depth
        ///   Depth map pointer for output
        /// @param dfs_disparity
        ///   DFS disparity parameters pointer, can be null
        //------------------------------------------------------------------------------
        virtual bool calculateDepth(uint8_t* imgL, uint8_t* imgR, float* depth, rvDFSDisparity* dfs_disparity=nullptr) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and get point cloud
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
        /// @param pointCloud
        ///   point cloud pointer to pointer for output
        /// @param dfs_disparity
        ///   DFS disparity parameters pointer, can be null
        //------------------------------------------------------------------------------
        virtual bool calculatePointCloud(uint8_t* imgL, uint8_t* imgR, PointCloudType* pointCloud, rvDFSDisparity* dfs_disparity=nullptr) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and get point cloud with color image
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
        /// @param pointCloud
        ///   point cloud pointer to pointer for output
        /// @param dfs_disparity
        ///   DFS disparity parameters pointer, can be null
        //------------------------------------------------------------------------------
        virtual bool calculatePointCloudColor(uint8_t* imgL, uint8_t* imgR, PointCloudColorType* pointCloud, rvDFSDisparity* dfs_disparity=nullptr) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and get disparity/depth/point cloud
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
        /// @param disparities
        ///   Disparity map pointer for output
        /// @param depth
        ///   Depth map pointer for output
        /// @param pc
        ///   point cloud pointer to pointer for output
        /// @param pcc
        ///   point cloud color pointer to pointer for output. Another format.
        /// @param dfs_disparity
        ///   DFS disparity parameters pointer, can be null
        //------------------------------------------------------------------------------
        virtual bool calculateDispDepthPointCloud(uint8_t* imgL, uint8_t* imgR, float* disparities, float* depth, PointCloudType* pc, PointCloudColorType* pcc, rvDFSDisparity* dfs_disparity=nullptr) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and get disparity/depth/point cloud with color image
        /// @param imgL
        ///   Left image pointer
        /// @param imgR
        ///   Right image pointer
        /// @param disparities
        ///   Disparity map pointer for output
        /// @param depth
        ///   Depth map pointer for output
        /// @param pc
        ///   Point cloud pointer for output
        /// @param pcc
        ///   point cloud color pointer to pointer for output. Another format.
        /// @param rectL
        ///   rectified left image for output
        /// @param rectR
        ///   rectified right image for output
        /// @param dfs_disparity
        ///   DFS disparity parameters pointer, can be null
        //------------------------------------------------------------------------------
        virtual bool calculateDfsAllInfo(uint8_t* imgL, uint8_t* imgR, float* disp, float* depth, PointCloudType* pc, PointCloudColorType* pcc, uint8_t* rectL=nullptr, uint8_t* rectR=nullptr, rvDFSDisparity* dfs_disparity=nullptr)=0;

        /// @brief 
        ///     Get point cloud result under user-defined coordinate.
        /// @param imgL 
        ///     Input left image pointer
        /// @param imgR 
        ///     Input right image pointer
        /// @param pointCloud 
        ///     Output point cloud in user coordinate.
        /// @param U2CMat 
        ///     Transform mat from user to left camera. Mat shape is 3*4. 
        virtual bool calculatePointCloudInUserCoordinate(uint8_t* imgL, uint8_t* imgR, PointCloudType* pointCloud, const float* U2CMat, rvDFSDisparity* dfs_disparity=nullptr) =0;

        /// @brief 
        ///     Get point cloud result under user-defined coordinate.
        /// @param imgL 
        ///     Input left image pointer
        /// @param imgR 
        ///     Input right image pointer
        /// @param pointCloud 
        ///     Output point cloud color data in user coordinate.
        /// @param U2CMat 
        ///     Transform mat from user to left camera. Mat shape is 3*4. 
        virtual bool calculatePointCloudColorInUserCoordinate(uint8_t* imgL, uint8_t* imgR, PointCloudColorType* pointCloud, const float* U2CMat, rvDFSDisparity* dfs_disparity=nullptr) =0;

        /// @brief 
        ///     Get point cloud result under user-defined coordinate.
        /// @param imgL 
        ///     Input left image pointer
        /// @param imgR 
        ///     Input right image pointer
        /// @param pointCloud 
        ///     Output point cloud in user coordinate.
        /// @param offset3
        ///     xyz offset from user coordinate to left camera coordinate. len is 3. 
        virtual bool calculatePointCloudAddOffset3(uint8_t* imgL, uint8_t* imgR, PointCloudType* pointCloud, const float* offset3, rvDFSDisparity* dfs_disparity=nullptr) =0;

        /// @brief 
        ///     Get point cloud result under user-defined coordinate.
        /// @param imgL 
        ///     Input left image pointer
        /// @param imgR 
        ///     Input right image pointer
        /// @param pointCloud 
        ///     Output point cloud color data in user coordinate.
        /// @param offset3 
        ///     xyz offset from user coordinate to left camera coordinate. len is 3. 
        virtual bool calculatePointCloudColorAddOffset3(uint8_t* imgL, uint8_t* imgR, PointCloudColorType* pointCloud, const float* offset3, rvDFSDisparity* dfs_disparity=nullptr) =0;

        /// @brief 
        ///     transform point cloud data from source inPC to dst outPC.
        /// @param inPC
        ///     Input point cloud data.
        /// @param outPC
        ///     Output point cloud data.
        /// @param U2CMat 
        ///     Transform mat from inPC to outPC. Mat shape is 3*4.   
        virtual bool transformPointCloud(PointCloudType* inPC, PointCloudType* outPC, const float* U2CMat)=0;

        /// @brief 
        ///     transform point cloud data from source inPCC to dst outPCC.
        /// @param inPCC 
        ///     Input point cloud color data.
        /// @param outPCC 
        ///     Output point cloud color data.
        /// @param U2CMat 
        ///     Transform mat from inPCC to outPCC. Mat shape is 3*4.       
        virtual bool transformPointCloudColor(PointCloudColorType* inPCC, PointCloudColorType* outPCC, const float* U2CMat)=0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Convert disparity map to point cloud
        /// @param disparities
        ///   depth map pointer
        /// @param pointCloud
        ///   point cloud pointer to pointer for output
        //------------------------------------------------------------------------------
        virtual bool disparity2PointCloud(float* disparities, PointCloudType* pointCloud)=0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Convert disparity map to point cloud color data
        /// @param disparities
        ///   depth map pointer
        /// @param pointCloud
        ///   point cloud color pointer to pointer for output
        //------------------------------------------------------------------------------
        virtual bool disparity2PointCloudColor(uint8_t* rectL, float* disparities, PointCloudColorType* pointCloud)=0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and convert depth map to point cloud
        /// @param depth
        ///   depth map pointer
        /// @param pointCloud
        ///   point cloud pointer to pointer for output
        //------------------------------------------------------------------------------
        virtual bool depth2PointCloud(float* depth, PointCloudType* pointCloud) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and convert depth map to point cloud with gray color
        /// @param imgRectL
        ///   rectified left image pointor
        /// @param depth
        ///   depth map pointer
        /// @param pointCloud
        ///   point cloud pointer to pointer for output
        //------------------------------------------------------------------------------
        virtual bool depth2PointCloudColor(uint8_t* imgRectL, float* depth, PointCloudColorType* pointCloud) = 0;


        //------------------------------------------------------------------------------
        /// @detailed
        ///     Set region of interest
        /// @param X
        ///   Upper left point coordinate X
        /// @param Y
        ///   Upper left point coordinate Y
        /// @param width
        ///   width of the region
        /// @param height
        ///   height of the region
        //------------------------------------------------------------------------------
        virtual void setROI(int X, int Y, int width, int height) = 0;
        
        
        //------------------------------------------------------------------------------
        /// @detailed
        ///     Get rectified left and right images
        /// @param imgRectL
        ///   Left rectified image pointer
        /// @param imgRectR
        ///   Right rectified image pointer
        //------------------------------------------------------------------------------
        virtual void getRectImages(uint8_t* imgRectL, uint8_t* imgRectR) = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     Get rectified camera parameters
        /// @return
        ///   return rvStereoCamera parameters
        //-----------------------------------------------------------------------------
        virtual rvStereoCamera getRectifiedCameraParameter() = 0;

        //------------------------------------------------------------------------------
        /// @detailed
        ///     De-initialize 
        //------------------------------------------------------------------------------
        virtual void deInitialize() = 0;
    };

}  // namespace rv_dfs

#endif
