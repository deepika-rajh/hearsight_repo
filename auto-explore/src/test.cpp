/*****************************************************************************
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include <cstdio>
#include<unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <time.h>

#include "explorer.h"

using std::placeholders::_1;
explorer::Exploration * rvAMobj;

class AEwarper : public rclcpp::Node
{
public:
    AEwarper()
    : Node("aenode")
    {
        //time(&aeBegin);
        aeBegin = time((time_t *)NULL);

        //s1:initialization
        std::string confPath;
        confPath.append("/data/misc/auto-explore/Configuration/aeconfiguration.yaml");
        RCLCPP_INFO(this->get_logger(), "root configuration path is : %s\n", confPath.c_str());

        rvAMobj = new explorer::Exploration(confPath);

        //s2:call the main function
        this->turtle_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/commands/velocity", 10);
        this->goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        this->isProcessedRotate();

        //s3: subscription the map and pose
        tfbuffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface());
        tfbuffer_->setCreateTimerInterface(timer_interface);
        tflistener_ = std::make_shared<tf2_ros::TransformListener>(*tfbuffer_);
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&AEwarper::topic_map_callback, this, _1));
    }

    time_t aeBegin;
    time_t aeEnd;

    // Callback to register with tf2_ros::MessageFilter to be called when transforms are available
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    std::shared_ptr<tf2_ros::Buffer> tfbuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tflistener_;

    bool isProcessedRotate()
    {
        float rVelocity_ =0.5f;

        rclcpp::Rate loop_rate(10);
        float rotateAngle = 0;
        unsigned int cnt = 0;
        while (true && cnt++ < 30000)
        {
            geometry_msgs::msg::Twist vel_msg;
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = rVelocity_;

            turtle_vel_pub_->publish(vel_msg);

            loop_rate.sleep();
            rotateAngle += 0.1 * rVelocity_;
            if (rotateAngle > 6.2f)
            {
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0f;

                turtle_vel_pub_->publish(vel_msg);
                break;
            }
        }

        sleep(3); //10
        return true;
    }

    void topic_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
    {
        if(rvAMobj == NULL)
            return;

        // determine the robot pose in map through tf
        geometry_msgs::msg::TransformStamped transformStamped;

        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
        try
        {
            rclcpp::Duration transform_tolerance(4000000);
            transformStamped = tfbuffer_->lookupTransform("map", "base_link", tf2::TimePoint(), tf2::durationFromSec(1.0));
        }
        catch (tf2::LookupException & ex)
        {
            RCLCPP_INFO(this->get_logger(), "transform from base_link to map not ready");
            return;
        }

        geometry_msgs::msg::PoseStamped robotPose;
        robotPose.pose.position.x = transformStamped.transform.translation.x;
        robotPose.pose.position.y = transformStamped.transform.translation.y;
        robotPose.pose.position.z = transformStamped.transform.translation.z;
        robotPose.pose.orientation.x = transformStamped.transform.rotation.x;
        robotPose.pose.orientation.y = transformStamped.transform.rotation.y;
        robotPose.pose.orientation.z = transformStamped.transform.rotation.z;
        robotPose.pose.orientation.w = transformStamped.transform.rotation.w;

        // get the pose and status
        AEPOSE goal;
        STATUS result;
        rvAMobj->makePlan(robotPose, map, goal, result);

        // do the execute
        // send goal and action out
        rclcpp::Time now;
        geometry_msgs::msg::PoseStamped goal2mb;
        PointAE target_position = goal.robotPos;
        RCLCPP_INFO(this->get_logger(), "PURSUEGOAL=0, BACKTOORIGIN, ROTATE, AGAIN, HOLDON\n");
        RCLCPP_INFO(this->get_logger(), "current status is %d ", result);
        RCLCPP_INFO(this->get_logger(), "current goal is %f %f\n", target_position.x, target_position.y);
        switch (result)
        {
            case PURSUEGOAL:
                goal2mb.pose.position.x = goal.robotPos.x;
                goal2mb.pose.position.y = goal.robotPos.y;
                goal2mb.pose.position.z = 0.0f;
                goal2mb.pose.orientation.w = 1.0f;
                goal2mb.header.frame_id = "map";
                goal2mb.header.stamp = now;
                goal_pub_->publish(goal2mb);
                break;
            case BACKTOORIGIN: // would be changed as the PURSUEGOAL code if having master to call AE due to master would call stop() to stop AE
                goal2mb.pose.position.x = goal.robotPos.x;
                goal2mb.pose.position.y = goal.robotPos.y;
                goal2mb.pose.position.z = 0.0f;
                goal2mb.pose.orientation.w = 1.0f;
                goal2mb.header.frame_id = "map";
                goal2mb.header.stamp = now;
                goal_pub_->publish(goal2mb);
                rvAMobj->status_ = 1;
                delete rvAMobj;
                rvAMobj = NULL;
                aeEnd = time((time_t *)NULL);

                printf("AE start time is %d, end time is %d, duration time is %d\n", (int)aeBegin, (int)aeEnd, (int)(aeEnd-aeBegin));

                RCLCPP_INFO(this->get_logger(), "The original pose as final goal is sent. Exploration stopped.\n");
                break;

            case ROTATE:
                //TO DO
                //isProcessedRotate();
                break;

            case AGAIN:
                //TO DO
                //makePlan();
                break;

            case HOLDON:
                //TO DO
                sleep(3);
                break;
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AEwarper>());
    rclcpp::shutdown();

    return 0;
}
