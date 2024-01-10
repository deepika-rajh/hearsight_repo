/*****************************************************************************
@copyright
Copyright (c) 2020-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/


#ifndef RV_DFS_BASE_H
#define RV_DFS_BASE_H
/***************************************************************************//**
@file
    rv_dfs_base.h

@detailed
    DFS base class

@section Overview
    This feature receives stereo image and generate disparity map, depth map,
    and/or point cloud data

@internal
    Copyright 2021-2023 Qualcomm Technologies, Inc.  All rights reserved.
    Confidential & Proprietary.
*******************************************************************************/

#include <string>
#include <type_traits>
#include "rvDFS.h"

namespace rv_dfs
{
    template<typename T>
    using SignedT = typename std::conditional<
            std::is_integral<T>::value,
            std::make_signed<T>,
            std::common_type<T>>::type::type;

    //point cloud data definition is 
    //X axis, into the screen
    //Y axis, from right to left
    //Z axis, from bottom to top
    //The unit is milli-meter
    //Only support float and int16_t
    template<typename T>
    using PointCloudType = std::vector<std::array<SignedT<T>,3>>;

    //Only support float and int16_t data type, which determines outputs,
    //including disparity, depth, point cloud and point cloud color, data type.
    //Other data type are not supported and behavier is not defined.
    //When data type is int16_t, the last 4 bits of disparity are fraction data.
    template<typename T>
    class DFSBase
    {
    public:
        //------------------------------------------------------------------------------
        /// @detailed
        ///        Initialization with extended parameters
        /// @param dfs_parameter
        ///    DFS parameters with extended information
        /// @param stereo_parameter
        ///    Intrinsic and extrinsic parameters of stereo camera
        /// @return
        ///    Returns True if success or False if failure
        //------------------------------------------------------------------------------
        virtual bool initialize(const rvDFSParameter& dfs_parameter, const rvStereoCamera& stereo_parameter) = 0;

                                        
        //------------------------------------------------------------------------------
        /// @detailed
        ///      Run rvDFS and get disparity/depth/point cloud
        /// @param in
        ///   Input data
        /// @param out
        ///   Output data
        /// @return
        ///   Returns True if success or False if failure
        //------------------------------------------------------------------------------
        virtual bool compute(const void* in, void* out) = 0;


        //------------------------------------------------------------------------------
        /// @detailed
        ///     Run rvDFS and convert depth map to point cloud
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
        virtual bool depth2PointCloud(const T* depth, PointCloudType<T>* pointCloud, rvStereoCamera* param, float* poseCameraInWorld) = 0;


        //------------------------------------------------------------------------------
        /// @detailed
        ///     Update new stereo camera intrinsic and extrinsic parameters
        /// @param param
        ///   New stereo camera intrinsic and extrinsic parameters
        /// @return
        ///   Returns True if success or False if failure
        //------------------------------------------------------------------------------
        virtual bool updateStereoCameraParam(rvStereoCamera& param) = 0;


        //------------------------------------------------------------------------------
        /// @detailed
        ///      Get rectified camera parameters
        /// @return
        ///    return rvStereoCamera parameters
        //-----------------------------------------------------------------------------
        virtual rvStereoCamera getRectCameraParam() = 0;


        //------------------------------------------------------------------------------
        /// @detailed
        ///      De-initialize 
        //------------------------------------------------------------------------------
        virtual void deInitialize() = 0;
    };

}  // namespace rv_dfs

#endif
