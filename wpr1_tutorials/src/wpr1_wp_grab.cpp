/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <waterplus_map_tools/Waypoint.h>
#include <waterplus_map_tools/GetWaypointByName.h>

static bool bGrabDone = false;
void GrabResultCB(const std_msgs::String::ConstPtr &msg)
{
    //ROS_WARN("[GrabResultCB] %s",msg->data.c_str());
    if(msg->data == "done")
    {
        bGrabDone = true;
    }
}

static bool bPassDone = false;
void PassResultCB(const std_msgs::String::ConstPtr &msg)
{
    //ROS_WARN("[PassResultCB] %s",msg->data.c_str());
    if(msg->data == "done")
    {
        bPassDone = true;
    }
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "wp_node");
    ros::NodeHandle nh;
    ros::ServiceClient cliGetWPName = nh.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    waterplus_map_tools::GetWaypointByName srvName;
    ros::Publisher behaviors_pub = nh.advertise<std_msgs::String>("/wpr1/behaviors", 30);
    ros::Subscriber grab_res_sub = nh.subscribe("/wpr1/grab_result", 30, GrabResultCB);
    ros::Subscriber pass_res_sub = nh.subscribe("/wpr1/pass_result", 30, PassResultCB);

    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;

    // 用for循环，依次完成三组相同的行为步骤（仅仅是货架航点名称不一样）
    for(int i=0;i<3;i++)
    {
        // 第一步 去往货架航点
        std::ostringstream stringStream;
        stringStream << (i+1);
        srvName.request.name = stringStream.str();
        if (cliGetWPName.call(srvName))
        {
            float x = srvName.response.pose.position.x;
            float y = srvName.response.pose.position.y;
            ROS_INFO("Waypoint: name = %s (%.2f,%.2f)", srvName.response.name.c_str(),x,y);
        }
        else
        {
            ROS_ERROR("Failed to call service get_waypoint_name");
            continue;
        }
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = srvName.response.pose;
        ac.sendGoal(goal);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            // 到达货架航点，开始抓取
            ROS_INFO("Arrived at %s!",srvName.response.name.c_str());
            bGrabDone = false;
            std_msgs::String behavior_msg;
            behavior_msg.data = "grab start";
            behaviors_pub.publish(behavior_msg);
        }
        else
        {
            continue;
        }
        // 等待抓取完成
        while(ros::ok() && bGrabDone == false)
        {
            ros::spinOnce();
        }

        // 第二步 去往垃圾桶
        srvName.request.name = "bin";
        if (cliGetWPName.call(srvName))
        {
            float x = srvName.response.pose.position.x;
            float y = srvName.response.pose.position.y;
            ROS_INFO("Waypoint: name = %s (%.2f,%.2f)", srvName.response.name.c_str(),x,y);
        }
        else
        {
            ROS_ERROR("Failed to call service get_waypoint_name");
            continue;
        }
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = srvName.response.pose;
        ac.sendGoal(goal);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            // 到达垃圾桶航点，扔垃圾
            ROS_INFO("Arrived at %s!",srvName.response.name.c_str());
            bPassDone = false;
            std_msgs::String behavior_msg;
            behavior_msg.data = "pass start";
            behaviors_pub.publish(behavior_msg);
        }
        else
        {
            continue;
        }

        // 等待扔垃圾完成
        while(ros::ok() && bPassDone == false)
        {
            ros::spinOnce();
        }
    }

    // 最后，去往exit航点，离开场地
    srvName.request.name = "exit";
    if (cliGetWPName.call(srvName))
    {
        float x = srvName.response.pose.position.x;
        float y = srvName.response.pose.position.y;
        ROS_INFO("Waypoint: name = %s (%.2f,%.2f)", srvName.response.name.c_str(),x,y);
    }
    else
    {
        ROS_ERROR("Failed to call service get_waypoint_name");
        return 0;
    }
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = srvName.response.pose;
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        // 到达exit航点，任务结束
        ROS_INFO("Arrived at %s!",srvName.response.name.c_str());
    }
}