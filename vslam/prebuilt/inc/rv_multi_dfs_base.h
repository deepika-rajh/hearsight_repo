/*****************************************************************************
Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
All rights reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef RV_MULTI_DFS_BASE_H
#define RV_MULTI_DFS_BASE_H

#include "rv_dfs_base.h"
#include <functional>

namespace rv_dfs {
//------------------------------------------------------------------------------
/// @detailed
///     Callback function to receive depth/point cloud/point cloud
///     color/disparity images. ThreadBase bind a user callback. The threadbase
///     will be blocked until callback is finished.
/// @param out
///   output data
//------------------------------------------------------------------------------
template <typename T>
using dfsCallback = std::function<void(rvDFSOutputParam *out)>;

//------------------------------------------------------------------------------
/// @detailed
///     callback function to post process raw disparity image
/// @param dispRaw
///   Pointer to raw disparity image
/// @param dispProc
///   Pointer to processed disaprity image, e.g. output
//------------------------------------------------------------------------------
template <typename T>
using dfsPostProcCb = std::function<void(T *dispRaw, T *dispProc)>;

template <typename T> class DFSThreadBase {
public:
  //------------------------------------------------------------------------------
  /// @detailed
  ///      Initialize for a new stereo camera
  /// @param streamID
  ///   ID of a stream provided by user, it should be unique for an DFS
  ///   instance, DFS algorithm uses the ID to determine which instance it
  ///   belongs to
  /// @param width
  ///   Width of input images
  /// @param height
  ///   Height of input images
  /// @param stride
  ///   Stride of input images
  /// @param dfs_mode
  ///   DFS mode
  /// @param dfs_parameter
  ///   DFS parameters
  /// @param stereo_parameter
  ///   Intrinsic and extrinsic parameters of stereo camera
  /// @param dfsCallback
  ///   Callback function to process dfs output
  /// @return
  ///   Returns True if success or False if failure
  //------------------------------------------------------------------------------
  virtual bool initialize(int streamID, const rvDFSParameter &dfs_parameter,
                          const rvStereoCamera &stereo_parameter,
                          dfsCallback<T> userCallback) = 0;

  //------------------------------------------------------------------------------
  /// @detailed
  ///     Run rvDFS and get disparity map
  /// @param streamID
  ///   ID of a stream, it should be unique for an DFS instance, DFS algorithm
  ///   uses the ID to determine which instance it belongs to
  /// @param in
  ///   Input data
  /// @param out
  ///   Output data
  /// @return
  ///   Returns True if success or False if failure
  //------------------------------------------------------------------------------
  virtual bool addImage(const int streamID, const rvDFSInputParam *in,
                        rvDFSOutputParam *out) = 0;

  //------------------------------------------------------------------------------
  /// @detailed
  ///     Get rectified camera parameters
  /// @param streamID
  ///   ID of a stream, it should be unique for an DFS instance, DFS algorithm
  ///   uses the ID to determine which instance it belongs to
  /// @return
  ///   return rvStereoCamera parameters
  //-----------------------------------------------------------------------------
  virtual rvStereoCamera getRectCameraParam(int streamID) = 0;

  //------------------------------------------------------------------------------
  /// @detailed
  ///     Update new stereo camera intrinsic and extrinsic parameters
  /// @param streamID
  ///   ID of a stream, it should be unique for an DFS instance, DFS algorithm
  ///   uses the ID to determine which instance it belongs to
  /// @param param
  ///   New stereo camera intrinsic and extrinsic parameters
  /// @return
  ///   Returns True if success or False if failure
  //------------------------------------------------------------------------------
  virtual bool updateStereoCameraParam(int streamID, rvStereoCamera param) = 0;

  //------------------------------------------------------------------------------
  /// @detailed
  ///     De-initialize
  /// @param streamID
  ///   ID of a stream, it should be unique for an DFS instance, DFS algorithm
  ///   uses the ID to determine which instance it belongs to
  /// @param disparities
  ///   Disparity map pointer for output
  //------------------------------------------------------------------------------
  virtual void deInitialize(int streamID) = 0;
};

} // namespace rv_dfs
#endif