/*****************************************************************************
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include <string>
#include <memory.h>
#include <explorer.h>
#include "rv.h"
#include "rvAE.h"

int RV_LOG_LEVEL = 0;
bool RV_STDERR_LOGGING = 0;
int debugLevel = 0;

namespace explorer
{
    Exploration::Exploration(std::string configurePath)
    {
        status_ = 0;
        init(configurePath);
        printf("Successfuly do exploration contrustion\n");
    }

    void Exploration::init(std::string configurePath)
    {
        pAEObj = rvAE_Initialize(configurePath.c_str());
        if (pAEObj == NULL)
        {
            printf("Failed to do the initializaiton of pAEObj\n");
            while (1) { ; }
        }

        return;
    }

    void Exploration::mapExpand(MapInfo &map)
    {
        unsigned int imgWidth = map.width_;
        unsigned int imgHeight = map.height_;

        unsigned int expansionRaduis = 4;

        if(imgWidth <= expansionRaduis*2-1 || imgHeight <= expansionRaduis*2-1)
            return;

        // expansion pixel indexs
        unsigned int num = (expansionRaduis*2-1)*(expansionRaduis*2-1);

        static std::vector<signed int> indexs;
        if(indexs.size() == 0)
        {
            for(unsigned int row = 0; row < expansionRaduis * 2 - 1; row++)
            {
                for(unsigned int col = 0; col <= expansionRaduis * 2 - 1; col++)
                {
                    signed int index = (row-expansionRaduis -1) * imgWidth + (col - expansionRaduis -1);
                    indexs.push_back(index);
                }
            }
        }

        unsigned char * mapData = new unsigned char[map.width_ * map.height_];
        memcpy(mapData, map.map_, sizeof(unsigned char) * imgWidth * imgHeight);

        for(unsigned int row = expansionRaduis; row < imgHeight - expansionRaduis; row++)
        {
            int indexRow = row * imgWidth;
            for(unsigned int col = expansionRaduis; col < imgWidth - expansionRaduis; col++)
            {
                int index = indexRow + col;

                unsigned char mapValue = map.map_[index];

                if(mapValue == 254)
                {
                    // do the expansion
                    for(unsigned int i = 0; i < num; i++)
                    {
                        unsigned int curIndex = indexs[i] + index;
                        if(curIndex <= 0 || curIndex > imgWidth * imgHeight)
                            continue;

                        mapData[curIndex] = 254;
                    }
                }
            }
        }

        memcpy(map.map_, mapData, sizeof(unsigned char) * imgWidth * imgHeight);
        delete []mapData;
    }

    void Exploration::makePlan(geometry_msgs::msg::PoseStamped &robotPose,
        const nav_msgs::msg::OccupancyGrid::ConstSharedPtr gridMap, AEPOSE &goal, STATUS &result)
    {
        AEPOSE curRobotPose;
        curRobotPose.robotPos.x = robotPose.pose.position.x;
        curRobotPose.robotPos.y = robotPose.pose.position.y;
        curRobotPose.robotPos.z = robotPose.pose.position.z;
        curRobotPose.rx = robotPose.pose.orientation.x;
        curRobotPose.ry = robotPose.pose.orientation.y;
        curRobotPose.rz = robotPose.pose.orientation.z;
        curRobotPose.rw = robotPose.pose.orientation.w;

        MapInfo map;
        map.width_ = gridMap->info.width;
        map.height_ = gridMap->info.height;

        unsigned char * mapData = new unsigned char[map.width_ * map.height_];
        map.map_ = mapData;

        map.mapResolution_ = gridMap->info.resolution;
        map.mapOriginX_ = gridMap->info.origin.position.x;
        map.mapOriginY_ = gridMap->info.origin.position.y;
        uint32_t mapsize = map.width_ * map.height_;

        memcpy(map.map_, &(gridMap->data[0]), sizeof(unsigned char)*map.width_*map.height_);

        // expand the occupied area to forbid the frontiers outside the wall or inside the wall
        mapExpand(map);

        // check if the robot's located in the inflation area or occupied area
        // if yes, make it to free for finding a way out to some accessiable goal
        int mx = (curRobotPose.robotPos.x - map.mapOriginX_) / map.mapResolution_ + 0.5f;
        int my = (curRobotPose.robotPos.y - map.mapOriginY_) / map.mapResolution_ + 0.5f;

        int index = my * map.width_ + mx;
        if(map.map_[index] == 253 || map.map_[index] == 254)
        {
            int win = 0.3f / map.mapResolution_; //0.2f is robot's radius
            for(int r = my - win; r <= my + win; r++)
            {
                if(r < 0 || r >= (int)map.width_)
                    continue;

                for(int c = mx - win; c <= mx +win; c++)
                {
                    if(c < 0 || c >= (int)map.height_)
                        continue;
    
                    int index = r * map.width_ + c;
                    map.map_[index] = 0; //set it free
                }
            }
        }

        // determine the goal and action for the current robot
        rvAE_determineGoal(pAEObj, map, curRobotPose, goal, result);
        delete mapData;

        return;
    }

    void Exploration::stop()
    {
        status_ = 1;
        printf("Exploration stopped.");
    }

}  // namespace explore
