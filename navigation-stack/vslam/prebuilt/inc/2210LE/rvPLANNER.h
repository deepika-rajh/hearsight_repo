/*****************************************************************************
@copyright
Copyright (c) 2021 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef RVPATHPLANNER_H
#define RVPATHPLANNER_H

/******************************************************************************
@file
   rvPLANNER.h

@detailed
   Robot Vision,
   Path Planner

@section Overview
   This feature receives grid map together with navigation goal, and generates applicable path if it exists.

*******************************************************************************/


//==============================================================================
// Includes
//==============================================================================

#include "rv.h"

//==============================================================================
// Declarations
//==============================================================================

#ifdef __cplusplus
extern "C"
{
#endif

   
   //==============================================================================
   /// @detailed
   ///     Path Planning.
   //==============================================================================
   typedef struct PLANNER rvPLANNER;


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Initialize a rvPLANNER object.
   /// @param parameters
   ///     Parameters for the path planning task.
   /// @return
   ///     Returns rvPLANNER object pointer if succeeded, and NULL if failed
   //------------------------------------------------------------------------------
   RV_API rvPLANNER* rvPLANNER_Initialize(rvPathPlanningParameters parameters);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Deinitialize a rvPLANNER object.
   /// @param pObj
   ///     Pointer to rvPLANNER object.
   //------------------------------------------------------------------------------
   RV_API void rvPLANNER_Deinitialize(rvPLANNER* pObj);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Make a path plan based on the given map and positions.
   /// @param pObj
   ///     Pointer to rvPLANNER object.
   /// @param gridMap
   ///     The grid map based on which to make the plan.
   /// @param width
   ///     Width of the grid map.
   /// @param height
   ///     Height of the grid map.
   /// @param origin
   ///     Position (in pixel) of origin-in-world-cooridnate in the grid map.
   /// @param from
   ///     The departure position.
   /// @param to
   ///     The destination to reach.
   /// @return
   ///     Returns number of steps to reach the goal. Negative values or zero mean the goal is unreachable.
   //------------------------------------------------------------------------------
   RV_API int rvPLANNER_MakePlan(rvPLANNER* pObj, const unsigned char* gridMap, const int width, const int height, const rvPixel2D origin, const rvPosition2D from, const rvPosition2D to);


   //------------------------------------------------------------------------------
   /// @detailed
   ///     Get the planned path.
   /// @param pObj
   ///     Pointer to rvPLANNER object.
   /// @param path
   ///     Memory where to copy the planned path. Format is x1,y1,x2,y2,...... So size of path should be 2 x number of steps (return from rvPLANNER_MakePlan)
   /// @return
   ///     True if the path copied and false otherwise.
   //------------------------------------------------------------------------------
   RV_API bool rvPLANNER_GetPlan(rvPLANNER* pObj, float* path);


#ifdef __cplusplus
}
#endif

#endif /* ifndef RVPATHPLANNER_H */
