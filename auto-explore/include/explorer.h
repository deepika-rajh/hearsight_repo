/*****************************************************************************
Copyright (c) 2022-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef NAV_EXPLORATIONrr_H_
#define NAV_EXPLORATIONrr_H_

#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rvAE.h"

namespace explorer
{
    class Exploration
    {
    public:
        // construction function, to initialize the related parameters through the launch file
        Exploration(std::string configurePath);

        void makePlan(geometry_msgs::msg::PoseStamped &robotPose, const nav_msgs::msg::OccupancyGrid::ConstSharedPtr map,
            AEPOSE &goal, STATUS &result);
        // stop AE for some unnormal cases, such as no predefined command called by the client
        void stop();

        // represent the map building status: -1: fail; 0: ongoing; 1: success
        int status_;

    private:
        //frontier_exploration::GoalDetection search_;
        rvAE* pAEObj;

        void init(std::string configPath);
        void mapExpand(MapInfo &map);
    };
}
#endif
