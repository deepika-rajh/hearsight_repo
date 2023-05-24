/*****************************************************************************
@copyright
Copyright (c) 2020-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/


#ifndef _RVDFS_H_
#define _RVDFS_H_

/***************************************************************************//**
@file
   rvDFS.h

@detailed
   Robot Vision,
   Depth From Stereo (DFS)
*******************************************************************************/


//==============================================================================
// Includes
//==============================================================================

#include <rv.h>
#include <array>

//==============================================================================
// Declarations
//==============================================================================

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct DFSBase rvDFS;

    typedef enum
    {
        RV_DFS_CVP = 0,                         //CVP hardware mode, only valid with QRB5165 and QCS8550
        RV_DFS_COVERAGE,                        //CPU solution, coverage mode
        RV_DFS_SPEED,                           //OpenCL solution, speed mode, fastest mode
        RV_DFS_ACCURACY,                        //CPU solution, accuracy mode
        RV_DFS_NORMAL,                         //OpenCL solution, balance mode, balance between coverage and speed
    }rvDFSMode;

    typedef struct _rvDFSDisparity
    {
        int minDisparity;                       //Minimum disparity level to search
        int numDisparityLevels;                 //Number of disparity levels
    }rvDFSDisparity;

    typedef struct _rvDFSDepthRange
    {
        float minDepth;                         //Minimum depth to search
        float maxDepth;                         //Maximum depth to search
    }rvDFSDepthRange;

    typedef struct _rvDFSParameter
    {
        rvDFSMode   mode;                       //DFS mode
        rvDFSDisparity  disparity;              //Number of disparity levels
        rvDFSDepthRange depthRange;             //Depth range to search, user must input camera extrinsic parameters
        int filterWidth=0;                      //Width of filter
        int filterHeight=0;                     //Height of filter
        bool doRectification=false;             //Indicate if rectification is needed before DFS
        bool useDisp=true;                      //The algorithm searches based on disparity range. Users can provide either disparity or depthRange and set this item to indicate which one will be used.
    }rvDFSParameter;


    //point cloud data definition is 
    //X axis, into the screen
    //Y axis, from right to left
    //Z axis, from bottom to top
    typedef std::vector<std::array<float,3>> PointCloudType;

    typedef std::vector<std::array<float,6>> PointCloudColorType;

    //------------------------------------------------------------------------------
    /// @detailed
    ///      Initialize rvDFS
    /// @param dfs_mode
    ///   DFS mode
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
    ///   Pointer to DFS object
    ///   Returns NULL if failed
    //------------------------------------------------------------------------------
    RV_API rvDFS* rvDFS_Initialize(int width, int height, int stride,
        const rvDFSParameter dfs_parameter, const rvStereoCamera stereo_parameter);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Get rectified camera parameters
    /// @param pHandle
    ///   Handle of rvDFS
    /// @return 
    ///   return rvStereoCamera parameters
    //------------------------------------------------------------------------------
    rvStereoCamera RV_API rvDFS_GetRectifiedCameraParameter(rvDFS* pHandle);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Run rvDFS and get disparity map
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param imgL
    ///   Left image pointer
    /// @param imgR
    ///   Right image pointer
    /// @param disparityMap
    ///   Disparity map pointer for output
    /// @return
    ///   Returns True if success or False if failure
    //------------------------------------------------------------------------------
    bool RV_API rvDFS_CalculateDisparity(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, float* disparityMap);

        
    //------------------------------------------------------------------------------
    /// @detailed
    ///     Run rvDFS and get disparity map with new disparity range
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param imgL
    ///   Left image pointer
    /// @param imgR
    ///   Right image pointer
    /// @param disparityMap
    ///   Disparity map pointer for output
    /// @param dfs_disparity
    ///   DFS disparity parameters pointer, can be null
    /// @return
    ///   Returns True if success or False if failure
    //------------------------------------------------------------------------------
    bool RV_API rvDFS_CalculateDisparityWithNewDisparityRange(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, float* disparityMap, rvDFSDisparity* dfs_disparity);
    

    //------------------------------------------------------------------------------
    /// @detailed
    ///     Run rvDFS and get depth map
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param imgL
    ///   Left image pointer
    /// @param imgR
    ///   Right image pointer
    /// @param depthMap
    ///   Depth map pointer for output
    /// @return
    ///   Returns True if success or False if failure
    //------------------------------------------------------------------------------
    bool RV_API rvDFS_CalculateDepth(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, float* depthMap);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Run rvDFS and get depth map with new disparity range
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param imgL
    ///   Left image pointer
    /// @param imgR
    ///   Right image pointer
    /// @param depthMap
    ///   Depth map pointer for output
    /// @param dfs_disparity
    ///   DFS disparity parameters pointer, can be null
    /// @return
    ///   Returns True if success or False if failure
    //------------------------------------------------------------------------------
    bool RV_API rvDFS_CalculateDepthWithNewDisparityRange(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, float* depthMap, rvDFSDisparity* dfs_disparity);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Run rvDFS and get point cloud
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param imgL
    ///   Left image pointer
    /// @param imgR
    ///   Right image pointer
    /// @param pointCloud
    ///   Point cloud pointer for output
    /// @return
    ///   Returns True if success or False if failure
    //------------------------------------------------------------------------------
    bool RV_API rvDFS_CalculatePointCloud(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, PointCloudType* pointCloud);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Run rvDFS and get point cloud with new disparity range
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param imgL
    ///   Left image pointer
    /// @param imgR
    ///   Right image pointer
    /// @param pointCloud
    ///   Point cloud pointer for output
    /// @param dfs_disparity
    ///   DFS disparity parameters pointer, can be null
    /// @return
    ///   Returns True if success or False if failure
    //------------------------------------------------------------------------------
    bool RV_API rvDFS_CalculatePointCloudWithNewDisparityRange(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, PointCloudType* pointCloud, rvDFSDisparity* dfs_disparity);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Run rvDFS and get point cloud fused with rectified left gray image
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param imgL
    ///   Left image pointer
    /// @param imgR
    ///   Right image pointer
    /// @param pointCloud
    ///   Point cloud color pointer for output
    /// @return
    ///   Returns True if success or False if failure
    //------------------------------------------------------------------------------
    bool RV_API rvDFS_CalculatePointCloudColor(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, PointCloudColorType* pointCloud);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Run rvDFS and get point cloud fused with rectified left gray image with new disparity range
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param imgL
    ///   Left image pointer
    /// @param imgR
    ///   Right image pointer
    /// @param pointCloud
    ///   Point cloud color pointer for output
    /// @param dfs_disparity
    ///   DFS disparity parameters pointer, can be null
    /// @return
    ///   Returns True if success or False if failure
    //------------------------------------------------------------------------------
    bool RV_API rvDFS_CalculatePointCloudColorWithNewDisparityRange(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, PointCloudColorType* pointCloud, rvDFSDisparity* dfs_disparity);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Run rvDFS and get disparity/depth/point cloud
    /// @param pHandle
    ///   Handle of rvDFS
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
    /// @return
    ///   Returns True if success or False if failure
    //------------------------------------------------------------------------------
    bool RV_API rvDFS_CalculateDispDepthPointCloud(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, float* disparities, float* depth, PointCloudType* pc, PointCloudColorType* pcc);

    //------------------------------------------------------------------------------
    /// @detailed
    ///     Run rvDFS and get disparity/depth/point cloud with new disparity range
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param imgL
    ///   Left image pointer
    /// @param imgR
    ///   Right image pointer
    /// @param disparityMap
    ///   Disparity map pointer for output
    /// @param depthMap
    ///   Depth map pointer for output
    /// @param pc
    ///   Point cloud pointer for output
    /// @param pcc
    ///   point cloud color pointer to pointer for output. Another format.
    /// @param dfs_disparity
    ///   DFS disparity parameters pointer, can be null
    /// @return
    ///   Returns True if success or False if failure
    //------------------------------------------------------------------------------
    bool RV_API rvDFS_CalculateDispDepthPointCloudWithNewDisparityRange(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, float* disparities, float* depth, PointCloudType* pc, PointCloudColorType* pcc, rvDFSDisparity* dfs_disparity);

    //------------------------------------------------------------------------------
    /// @detailed
    ///   run rvDFS and get all outputs including rectified left image, disparity, depth, point cloud data with pc and pcc formats.
    /// @param pHandle
    ///   Handle of rvDFS
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
    /// @param rectLeft
    ///   rectified left image for output
    /// @param rectRight
    ///   rectified right image for output
    /// @param dfs_disparity
    ///   DFS disparity parameters pointer, can be null
    /// @return
    ///   Returns True if success or False if failure
    //------------------------------------------------------------------------------
    bool RV_API rvDFS_CalculateDfsAllInfo(uint8_t* imgL, uint8_t* imgR, float* disp, float* depth, PointCloudType* pc, PointCloudColorType* pcc, uint8_t* rectL, uint8_t* rectR, rvDFSDisparity* dfs_disparity);

    //------------------------------------------------------------------------------
    /// @brief 
    ///     Get point cloud result under user-defined coordinate.
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param imgL 
    ///     Input left image pointer
    /// @param imgR 
    ///     Input right image pointer
    /// @param pointCloud 
    ///     Output point cloud in user coordinate.
    /// @param U2CMat 
    ///     Transform mat, shape 3*4. 
    /// @param dfs_disparity
    ///   DFS disparity parameters pointer, can be null
    /// @return 
    ///     Return true if success, otherwise false.
    //------------------------------------------------------------------------------
        bool RV_API rvDFS_CalculatePointCloudInUserCoordinate(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, PointCloudType* pointCloud, const float* U2CMat, rvDFSDisparity* dfs_disparity);

    //------------------------------------------------------------------------------
    /// @brief 
    ///     Get point cloud result under user-defined coordinate.
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param imgL 
    ///     Input left image pointer
    /// @param imgR 
    ///     Input right image pointer
    /// @param pointCloud 
    ///     Output point cloud color data in user coordinate.
    /// @param U2CMat 
    ///     Transform mat, shape 3*4. 
    /// @param dfs_disparity
    ///   DFS disparity parameters pointer, can be null
    /// @return 
    ///     Return true if success, otherwise false.
    //------------------------------------------------------------------------------
        bool RV_API rvDFS_CalculatePointCloudColorInUserCoordinate(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, PointCloudType* pointCloud, const float* U2CMat, rvDFSDisparity* dfs_disparity);

    //------------------------------------------------------------------------------
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
    //------------------------------------------------------------------------------
        bool RV_API rvDFS_CalculatePointCloudAddOffset3(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, PointCloudType* pointCloud, const float* offset3, rvDFSDisparity* dfs_disparity);

    //------------------------------------------------------------------------------
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
    //------------------------------------------------------------------------------
        bool RV_API rvDFS_CalculatePointCloudColorAddOffset3(rvDFS* pHandle, uint8_t* imgL, uint8_t* imgR, PointCloudColorType* pointCloud, const float* offset3, rvDFSDisparity* dfs_disparity);

    //------------------------------------------------------------------------------
    /// @brief 
    ///     transform point cloud data from source inPC to dst outPC.
    /// @param inPC
    ///     Input point cloud data.
    /// @param outPC
    ///     Output point cloud data.
    /// @param U2CMat 
    ///     Transform mat from inPC to outPC. Mat shape is 3*4.
    //------------------------------------------------------------------------------
        bool RV_API rvDFS_TransformPointCloud(rvDFS* pHandle, PointCloudType* inPC, PointCloudType* outPC, const float* U2CMat);

    //------------------------------------------------------------------------------
    /// @brief 
    ///     transform point cloud data from source inPCC to dst outPCC.
    /// @param inPCC 
    ///     Input point cloud color data.
    /// @param outPCC 
    ///     Output point cloud color data.
    /// @param U2CMat 
    ///     Transform mat from inPCC to outPCC. Mat shape is 3*4.
    //------------------------------------------------------------------------------
    bool RV_API rvDFS_TransformPointCloudColor(rvDFS* pHandle, PointCloudColorType* inPCC, PointCloudColorType* outPCC, const float* U2CMat);

    /// @brief 
    ///     get point cloud result by disparity input.
    /// @param pHandle
    ///     Handle of rvDFS
    /// @param disparities
    ///     input float disparities
    /// @param pointCloud
    ///     output point cloud result.
    /// @return
    ///     Return True if success or False if it fails.
    bool RV_API rvDFS_Disparity2PointCloud(rvDFS* pHandle, float* disparities, PointCloudType* pointCloud);

    /// @brief 
    ///     get point cloud color result by disparity input.
    /// @param pHandle
    ///     Handle of rvDFS
    /// @param rectL
    ///     input rect left image 
    /// @param disparities
    ///     input float disparities
    /// @param pointCloud
    ///     output point cloud color result.
    /// @return
    ///     Return True if success or False if it fails.
    bool RV_API rvDFS_Disparity2PointCloudColor(rvDFS* pHandle, uint8_t* rectL, float* disparities, PointCloudColorType* pointCloud);

    //------------------------------------------------------------------------------
    /// @detailed
    ///     Convert depth map to point cloud
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param depthMap
    ///   Depth map pointer
    /// @param pointCloud
    ///   Point cloud pointer for output
    //------------------------------------------------------------------------------
    bool RV_API rvDFS_Depth2PointCloud(rvDFS* pHandle, float* depthMap, PointCloudType* pointCloud);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Convert depth map to point cloud color
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param imgRectL
    ///   rectified left image pointer
    /// @param depthMap
    ///   Depth map pointer
    /// @param pointCloud
    ///   Point cloud color pointer for output
    //------------------------------------------------------------------------------
    bool RV_API rvDFS_Depth2PointCloudColor(rvDFS* pHandle, uint8_t* imgRectL, float* depthMap, PointCloudColorType* pointCloud);


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
    void RV_API rvDFS_SetROI(rvDFS* pHandle, int X, int Y, int width, int height);
        
        
    //------------------------------------------------------------------------------
    /// @detailed
    ///     Get rectified left and right images
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param imgRectL
    ///   Left rectified image pointer
    /// @param imgRectR
    ///   Right rectified image pointer
    //------------------------------------------------------------------------------
    void RV_API rvDFS_GetRectImages(rvDFS* pHandle, uint8_t* imgRectL, uint8_t* imgRectR);

    //------------------------------------------------------------------------------
    /// @detailed
    ///     Deinitialize DFS object
    /// @param pHandle
    ///   Handle of rvDFS
    //------------------------------------------------------------------------------
    void RV_API rvDFS_Deinitialize(rvDFS* pHandle);



#ifdef __cplusplus
}
#endif


#endif
