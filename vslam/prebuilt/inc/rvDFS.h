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

@internal
   Copyright 2021-2023 Qualcomm Technologies, Inc.  All rights reserved.
   Confidential & Proprietary.
*******************************************************************************/


//==============================================================================
// Includes
//==============================================================================

#include <rv.h>
#include <rvCamera.h>
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
        RV_DFS_COVERAGE=0,                                  //CPU solution, coverage mode
        RV_DFS_BALANCE,                                     //OpenCL solution, balance mode, balance between coverage and speed
        RV_DFS_SPEED,                                       //OpenCL solution, speed mode, fastest mode
        RV_DFS_CVP,                                         //CVP hardware mode, only valid with QRB5165 and QCS8550
    } rvDFSMode;

    typedef enum
    {
        RV_DFS_PP_BASIC=0,                                      // basic mode
        RV_DFS_PP_MEDIUM,                                       // advanced mode
        RV_DFS_PP_STRONG,                                       // strong mode, need specific customer code
        RV_DFS_PP_SUPREME,                                      // supreme mode, need specific customer code
    } rvDFSPPLevel;

    typedef struct
    {
        int32_t minDisparity;                               //Minimum disparity level to search
        int32_t numDisparityLevels;                         //Number of disparity levels
    } rvDFSDisparity;

    typedef struct
    {
        int32_t minDepth;                                   //Minimum depth to search, unit: mm
        int32_t maxDepth;                                   //Maximum depth to search, unit: mm
    } rvDFSDepthRange;                                      //Todo: determine the working disparity range by [minDepth, inf]. maxDepth only used to generate(filter) depth output

    typedef struct
    {
        uint32_t        version;                            //API version
        uint32_t        paramSize;                          //Parameter size
        rvImageSize     inputSize;                          //Input image size
        rvImageFormat   imgFormat = Y_ONLY_FORMAT;          //0:grayscale by default, we don't support other image format yet
        rvImageSize     outputSize;                         //Output size, different input and output size are supported by Speed and Normal modes
        rvDFSMode       mode;                               //DFS mode
        rvDFSDisparity  disparity;                          //Number of disparity levels
        rvDFSDepthRange depthRange;                         //Depth range to search, user must input camera extrinsic parameters
        int32_t         filterWidth = 0;                    //Todo: delete
        int32_t         filterHeight = 0;
        int32_t         maxHdrFrames = 0;                   //The max image pairs will be processed by HDR, up to 3, 
        bool            doRectification = false;            //Indicate if rectification is needed before DFS
        bool            useDisp = true;                     //The algorithm searches based on disparity range. Users can provide either disparity or depthRange and set this item to indicate which one will be used.
        bool            latestOnly=true;                    //Only valid for multiThreadDFS. We suggest to process the latest frames only for online application. Offline application, we suggest to set it FALSE so that every frames will be processed.
        bool            useIONMem = false;                  //Use ION memory or not, it may save one copy if input images are from ISP and in ION memory, only valid for Balance and Speed mode
        rvDFSPPLevel    ppLevel = RV_DFS_PP_BASIC;          //Postprocessing level
        int32_t         extInfoSize;                        //Extended information size in bytes
        char*           extInfo = nullptr;                  //Pointer to extended information, yml/json
    } rvDFSParameter;

    //Not supported yet
    typedef struct
    {
        uint32_t        version;                            //API version
        uint32_t        paramSize;                          //Parameter size
        float*          extrinsic = nullptr;                //Input (4x4 matrix?) ==> for rgb-d fusion
        rvImageSize     imageSize;                          //Input image size
        rvImageFormat   imgFormat;                          //Image format
    } rvRGBDParameter;    

    typedef struct
    {
        rvRoi2D*         roi = nullptr;                     //Region of intereset, which is only applicable to speed and balance modes. 
        rvDFSDisparity*  disparity = nullptr;               //DFS disparity parameters, including min and levels, numDisparityLevels = 0 indicates re-use current disparity parameters
        rvDFSPPLevel*    ppLevel = nullptr;                 //Runtime postprocessing level;
    } rvDFSParamRuntime;

    typedef struct
    {
        uint32_t            version;                         //0x00010000 (1.0)
        uint32_t            paramSize;                       //Parameter size
        uint32_t            numParams = 0;                   //How many rvDFSParamRuntime will be passed to DFS algorithm
        rvDFSParamRuntime*  dfsParam = nullptr;              //DFS parameters which can be changed online, can be nullptr if no changes
        float*              poseCameraInWorld = nullptr;     //Camera-in-world transformation matrix. The matrix is 3 x 4, which represents [R | T]
        struct
        {
            uint64_t          imgTimestamp;                 //Time stamp of input image pair that are supposed to be synchronized.
            uint8_t *         imgLeft = nullptr;            //Input left image
            int32_t           ionFDLeft = -1;               //ION file descriptor of left image, can be ignored for non-ION inputs
            uint8_t *         imgRight = nullptr;           //Input right image
            int32_t           ionFDRight = -1;              //ION file descriptor of right image, can be ignored for non-ION inputs
        } input;        
    } rvDFS_InputParam_V1_t;

// todo: add a simple in. (del 98-100)

    typedef struct
    {
        uint32_t            version;                        //0x00020000 (2.0)
        uint32_t            paramSize;                      //Parameter size
        uint32_t            numParams = 0;                  //How many rvDFSParamRuntime will be passed to DFS algorithm
        rvDFSParamRuntime*  dfsParam = nullptr;             //DFS parameters which can be changed online, can be nullptr if no changes
        float*              poseCameraInWorld = nullptr;    //Camera-in-world transformation matrix. The matrix is 3 x 4, which represents [R | T]
        struct
        {
            uint64_t      imgTimestamp;                     //Time stamp of input image pair that are supposed to be synchronized.
            uint32_t      imgFrames;                        //How many image pairs will be used in HDR, up to 3, 0:reuses previous frame
            uint8_t *     imgLeft[3]{nullptr};              //Input left images
            int32_t       ionFDLeft[3]{-1};                 //ION file descriptor of left image, can be ignored for non-ION inputs
            uint8_t *     imgRight[3]{nullptr};             //Input right images
            int32_t       ionFDRight[3]{-1};                //ION file descriptor of right image, can be ignored for non-ION inputs
	    } input;
    } rvDFS_InputParam_V2_t;

    //V3 is not supported yet
    typedef struct
    {
        uint32_t            version;                        //0x00030000 (3.0)
        uint32_t            paramSize;
        uint32_t            numParams = 0;                  //How many rvDFSParamRuntime will be passed to DFS algorithm
        rvDFSParamRuntime*  dfsParam = nullptr;             //DFS parameters which can be changed online, can be nullptr if no changes
        float*              poseCameraInWorld = nullptr;    //Camera-in-world transformation matrix. The matrix is 3 x 4, which represents [R | T]
        struct
        {
            uint64_t      imgTimestamp;                     //Time stamp of left image, DFS output will use this time stamp
            uint32_t      imgFrames;                        //How many image pairs will be used in HDR, up to 3
            uint8_t *     imgLeft[3]{nullptr};              //Input left images
            int32_t       ionFDLeft[3]{-1};                 //ION file descriptor of left image, can be ignored for non-ION inputs
            uint8_t *     imgRight[3]{nullptr};             //Input right images
            int32_t       ionFDRight[3]{-1};                //ION file descriptor of right image, can be ignored for non-ION inputs
	    } input;
        struct 
        {
            uint64_t      imgTimestamp;                     //Time stamp of color image
            uint32_t      imgFrames;                        //How many image pairs will be used in HDR, up to 3
            uint8_t *     img[3]{nullptr};                  //Input image
            int32_t       ionFDColor[3];                    //ION file descriptor of color image, can be ignored for non-ION inputs
        } color;
    } rvDFS_InputParam_V3_t;

    // include dim, roi as well so that rvDF_OutputParam_V1_t alone can be passed over to following layers
    typedef struct
    {
        uint32_t          version;                          //0x10010000 (1.0) by default
        uint32_t          paramSize;
        struct
        {
            uint32_t      width, height;
        } dim;                                              //Image dimension
        rvRoi2D           roi;                              //Region of interest
        uint64_t          imgTimestamp;                     //Time stamp of input left image
        void*             pUserContext = nullptr;           //Pointer to user's own context
        uint8_t*          imgL = nullptr;                   //Output, fill nullptr if not needed, hdr image if applicable
        uint8_t*          imgR = nullptr;                   //Output, fill nullptr if not needed, hdr image if applicable
        uint8_t*          rectL = nullptr;                  //Output, fill nullptr if not needed
        uint8_t*          rectR = nullptr;                  //Output, fill nullptr if not needed
        uint32_t          mapDataType;                      //Disparity & depth map (0 : float, 1 : int16, 2 : int32, etc)
        void*             mapOfDisparity = nullptr;         //Output, fill nullptr if not needed
        void*             mapOfDepth = nullptr;             //Output, fill nullptr if not needed
        uint32_t          numPoints;                        //Number of 3d-points and colors
        void*             pointBuffer = nullptr;            //Output of {x,y,z}, fill nullptr if not needed
    } rvDFS_OutputParam_V1_t;


    //------------------------------------------------------------------------------
    /// @detailed
    ///      Initialize rvDFS
    /// @param dfs_mode
    ///   DFS mode
    /// @param dfs_parameter
    ///   DFS parameters with extended information
    /// @param stereo_parameter
    ///   Intrinsic and extrinsic parameters of stereo camera
    /// @return
    ///   Pointer to DFS object
    ///   Returns NULL if failed
    //------------------------------------------------------------------------------
    RV_API rvDFS* rvDFS_InitializeF32(const rvDFSParameter dfs_parameter, const rvStereoCamera stereo_parameter);
    RV_API rvDFS* rvDFS_InitializeU16(const rvDFSParameter dfs_parameter, const rvStereoCamera stereo_parameter);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Run rvDFS and get disparity/depth/point cloud/rectified images
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param in
    ///   Input data
    /// @param out
    ///   Output data
    /// @return
    ///   Returns True if success or False if failure
    //------------------------------------------------------------------------------
    RV_API bool rvDFS_ComputeF32(rvDFS* pHandle, const void* in, void* out);
    RV_API bool rvDFS_ComputeU16(rvDFS* pHandle, const void* in, void* out);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Update new stereo camera intrinsic and extrinsic parameters
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param param
    ///   New stereo camera intrinsic and extrinsic parameters
    /// @return
    ///   Returns True if success or False if failure
    //------------------------------------------------------------------------------
    RV_API bool rvDFS_UpdateStereoCameraParamF32(rvDFS* pHandle, rvStereoCamera param);
    RV_API bool rvDFS_UpdateStereoCameraParamU16(rvDFS* pHandle, rvStereoCamera param);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Get rectified camera parameters
    /// @param pHandle
    ///   Handle of rvDFS
    /// @return 
    ///   return rvStereoCamera parameters
    //------------------------------------------------------------------------------
    RV_API rvStereoCamera rvDFS_GetRectCameraParamF32(rvDFS* pHandle);
    RV_API rvStereoCamera rvDFS_GetRectCameraParamU16(rvDFS* pHandle);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Run rvDFS and convert depth map to point cloud
    /// @param pHandle
    ///   Handle of rvDFS
    /// @param depth
    ///   depth map pointer
    /// @param pointCloud
    ///   point cloud pointer to pointer for output
    /// @param param
    ///   rectified stereo camera parameters
    /// @param poseCameraInWorld
    ///   Camera-in-world transformation matrix. The matrix is 3 x 4, which represents [R | T]
    /// @return
    ///   Returns True if success or False if failure
    //------------------------------------------------------------------------------
    RV_API bool rvDFS_Depth2PointCloudF32(rvDFS* pHandle, const float* depth, void* pointCloud, rvStereoCamera* param, float* poseCameraInWorld);
    RV_API bool rvDFS_Depth2PointCloudU16(rvDFS* pHandle, const uint16_t* depth, void* pointCloud, rvStereoCamera* param, float* poseCameraInWorld);


    //------------------------------------------------------------------------------
    /// @detailed
    ///     Deinitialize DFS object
    /// @param pHandle
    ///   Handle of rvDFS
    //------------------------------------------------------------------------------
    RV_API void rvDFS_DeinitializeF32(rvDFS* pHandle);
    RV_API void rvDFS_DeinitializeU16(rvDFS* pHandle);


#ifdef __cplusplus
}
#endif


#endif
