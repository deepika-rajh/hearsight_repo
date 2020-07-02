/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef RV_H
#define RV_H
/***************************************************************************//**
@file
   rv.h

@detailed
   Common data structures and utilities for the Robot Vision SDK.

@mainpage
   Robot Vision SDK

@version
   1.0.0 (WIP1)

@section Overview
   QTI's Robot Vision SDK provides highly runtime optimized and state of
   the art robot vision algorithms to enable such features as localization,
   and mapping.  Some example features included are:
   - Visual Simultaneous Localization and Mapping (VSLAM) for robot
     localization and pose estimation.
   - Voxel Map (VM) for 3D depth fusion and mapping.

*******************************************************************************/
#include <stddef.h>
#include <stdbool.h>

#ifdef __GNUC__
#ifdef BUILDING_SO
// MACRO enables function to be visible in shared-library case.
#define RV_API __attribute__ ((visibility ("default")))
#else
// MACRO empty for non-shared-library case.
#define RV_API
#endif
#else

#ifdef RV_EXPORTS
// MACRO enables function to be visible in shared-library case.
#define RV_API __declspec(dllexport)
#else
// MACRO empty for non-shared-library case.
#define RV_API
#endif
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/***********************************
 * return RV SDK version string
 * ********************************/
RV_API const char* rvVersion( void );

#ifdef __cplusplus
}
#endif

#ifdef __ARM_NEON__
#include <arm_neon.h>
typedef float  float32_t;
typedef double float64_t;
#else
#include <stdint.h>
typedef float  float32_t;
typedef double float64_t;
#endif


#endif

