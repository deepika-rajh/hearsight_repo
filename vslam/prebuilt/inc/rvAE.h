/*****************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/


#ifndef RVGOALDETECTION_H
#define RVGOALDETECTION_H

/***************************************************************************//**
@file
   rvAE.h

@detailed
   Robot Vision. 
   To determine a goal for the robot automatically building the map 

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

    struct PointAE {
        double x;
        double y;
        double z;
    };


    struct AEPOSE {
        PointAE robotPos;
        double rx, ry, rz, rw;
    };


    struct Frontier {
        uint32_t size;
        double min_distance;
        double cost;
        PointAE initial;
        PointAE centroid;
        PointAE middle;
        std::vector<PointAE> points;
    };


    struct MapInfo {
        unsigned char* map_;
        unsigned int width_;
        unsigned int height_;
        float mapResolution_;
        float mapOriginX_;
        float mapOriginY_;
    };


    enum STATUS { PURSUEGOAL, BACKTOORIGIN, ROTATE, AGAIN, HOLDON };
    /************************************************************************//**
    @detailed
       Goal detection for robot automatically building the map
    ****************************************************************************/
    typedef struct rvAE rvAE;

    /************************************************************************//**
    @detailed
       Initialize rvAE.
    @param root_path
       The root path of configuration file
    @returns
       Pointer to rvAE object; returns NULL if failed.
    ****************************************************************************/
    RV_API rvAE* rvAE_Initialize(const char* root_path);

    /************************************************************************//**
    @detailed
       Determine the current goal given the map and the current robot pose
    @param pObj
       Pointer to rvAE object.
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
    RV_API bool rvAE_determineGoal(rvAE* pObj, const MapInfo map, const AEPOSE curPos, AEPOSE& goal, STATUS& result);




    RV_API bool rvAE_setBlackList(rvAE* pObj, PointAE curFrontierCenter);

#ifdef __cplusplus
}
#endif


#endif
