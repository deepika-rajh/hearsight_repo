/*****************************************************************************
 * @copyright
 * Copyright (c) 2021 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 * *******************************************************************************/

#ifndef MV_NAVMAP_H
#define MV_NAVMAP_H

/***************************************************************************//**
@file
   rvNAVMAP.h

@detailed
   Robot Vision,
   Navigation mapping

*******************************************************************************/


//==============================================================================
// Includes
//==============================================================================

#include "rv.h"

#ifdef __cplusplus
extern "C"
{
#endif

	//==============================================================================
	/// @detailed
	///     Navigation Mapping
	//==============================================================================
	typedef struct NAVMAP rvNAVMAP;

	//------------------------------------------------------------------------------
	/// @detailed
	///     Set map resolution from configuration file
	/// @param root_path
	///     Pass path of configuration file
	/// @return
	///     Return pointer of rvNAVMAP if succeeded, NULL if failed
	//------------------------------------------------------------------------------
	RV_API rvNAVMAP* rvNAVMAP_Initialize(const char* root_path);

	//------------------------------------------------------------------------------
	/// @detailed
	///     Delete the exsiting map file in memory if needed
	/// @param pObj
	///     Pointer of NAVMAP object
	//------------------------------------------------------------------------------
	RV_API void rvNAVMAP_Clearmap(rvNAVMAP* pObj);

	//------------------------------------------------------------------------------
	/// @detailed
	///     Release the buffer if needed
	/// @param pObj
	///     Pointer of NAVMAP object
	//------------------------------------------------------------------------------
	RV_API void rvNAVMAP_Deepreset(rvNAVMAP* pObj);

	//------------------------------------------------------------------------------
	/// @detailed
	///     Delete the object before stopping the program
	/// @param pObj
	///     Pointer of Pointer of NAVMAP object
	//------------------------------------------------------------------------------
	RV_API void rvNAVMAP_Deinitialize(rvNAVMAP** pObj);

	//------------------------------------------------------------------------------
	/// @detailed
	///     New map mode, create the directory for saving new map
	/// @param pObj
	///     Pointer of NAVMAP object
	/// @return
	///     True if succeeded, false if failed
	//------------------------------------------------------------------------------
	RV_API bool rvNAVMAP_Newmap(rvNAVMAP* pObj);

	//------------------------------------------------------------------------------
	/// @detailed
	///     Map loading, load and publish the 2D map given the directory
	/// @param pObj
	///     Pointer of NAVMAP object
	/// @return
	///     True if succeeded, false if failed
	//------------------------------------------------------------------------------
	RV_API bool rvNAVMAP_Load2Dmap(rvNAVMAP* pObj);

	//------------------------------------------------------------------------------
	/// @detailed
	///     Build map using keyframe info
	/// @param pObj
	///     Pointer of NAVMAP object
	/// @return
	///     True if succeeded, false if failed
	//------------------------------------------------------------------------------
	RV_API bool rvNAVMAP_BuildmapUsingKeyframeInfo(rvNAVMAP* pObj);

	//------------------------------------------------------------------------------
	/// @detailed
	///     Build map using online pose and frame
	/// @param pObj
	///     Pointer of NAVMAP object
	/// @param imageData
	///     Pass pointer of image buffer
	/// @param pose
	///     6DRT pose data
	/// @param cameraParam
	///     Camera configrations
	/// @return
	///     True if succeeded, false if failed
	//------------------------------------------------------------------------------
	RV_API bool rvNAVMAP_BuildmapUsingOnlineInfo(rvNAVMAP* pObj, const uint16_t * imageData, const rvPose6DRT pose, rvCameraConfiguration cameraParam);

	//------------------------------------------------------------------------------
	/// @detailed
	///     Build 2D map 
	/// @param pObj
	///     Pointer of NAVMAP object
	/// @return
	///     True if succeeded, false if failed
	//------------------------------------------------------------------------------
	RV_API bool rvNAVMAP_Build2Dmap(rvNAVMAP* pObj);

	//------------------------------------------------------------------------------
	/// @detailed
	///     Build 3D map
	/// @param pObj
	///     Pointer of NAVMAP object
	/// @return
	///     True if succeeded, false if failed
	//------------------------------------------------------------------------------
	RV_API bool rvNAVMAP_Build3Dmap(rvNAVMAP* pObj);

	//------------------------------------------------------------------------------
	/// @detailed
	///     Pause mapper to build map 
	/// @param pObj
	///     Pointer of NAVMAP object
	/// @return
	///     True if succeeded, false if failed
	//------------------------------------------------------------------------------
	RV_API bool rvNAVMAP_Pausetobuildmap(rvNAVMAP* pObj);

	//------------------------------------------------------------------------------
	/// @detail
	///     Save map 
	/// @param pObj
	///     Pointer of NAVMAP object
	/// @return
	///     True if succeeded, false if failed
	//------------------------------------------------------------------------------
	RV_API bool rvNAVMAP_Savemap(rvNAVMAP* pObj);

#ifdef __cplusplus
}
#endif

#endif
