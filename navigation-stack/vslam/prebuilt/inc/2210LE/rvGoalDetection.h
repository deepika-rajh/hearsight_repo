/*****************************************************************************
@copyright
Copyright (c) 2021 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/


#ifndef RVGOALDETECTION_H
#define RVGOALDETECTION_H

/***************************************************************************//**
@file
   rvGoalDetection.h

@detailed
   Robot Vision. 
   To determine a goal for the robot automatically building the map 

@section Overview 

*******************************************************************************/


//==============================================================================
// Defines
//==============================================================================


//==============================================================================
// Includes
//============================================================================== 
#include "rv.h" 

#ifdef __GNUC__
#ifdef BUILDING_SO
// MACRO enables function to be visible in shared-library case.
#define RV_API __attribute__ ((visibility ("default")))
#else
// MACRO empty for non-shared-library case.
#define RV_API
#endif
#else

#ifdef WIN32
// MACRO enables function to be visible in shared-library case.
#define RV_API __declspec(dllexport)
#else
// MACRO empty for non-shared-library case.
#define RV_API
#endif
#endif


//==============================================================================
// Declarations
//==============================================================================

#ifdef __cplusplus
extern "C"
{
#endif

    /************************************************************************//**
    @detailed
       Goal detection for robot automatically building the map
    ****************************************************************************/
    typedef struct rvGoalDetection rvGoalDetection;

    /************************************************************************//**
    @detailed
       Initialize rvGoalDetection.
    @param root_path
       The root path of configuration file  
    @returns
       Pointer to rvGoalDetection object; returns NULL if failed.
    ****************************************************************************/
    RV_API rvGoalDetection* rvGoalDetection_Initialize(const char* root_path); 

    /************************************************************************//**
    @detailed
       Determine the current goal given the map and the current robot pose
    @param pObj
       Pointer to rvGoalDetection object.
    @param map
       Input parameter of map information.
    @param curPos
       Input parameter of current robot position.
    @param goal
       Output parameter of the current determined goal <pose>.
    @param result
       Output parameter of the action.
    @returns
       Successful or not. If failed, means there is no goal and the map completed
    ****************************************************************************/
    RV_API bool rvGoalDetection_determineGoal(rvGoalDetection* pObj, const MapInfo map, const PointAE curPos, AEPOSE& goal, STATUS& result);
     



    RV_API bool rvGoalDetection_setBlackList(rvGoalDetection* pObj, PointAE curFrontierCenter);

#ifdef __cplusplus
}
#endif


#endif
