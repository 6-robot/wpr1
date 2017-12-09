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
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

static tf::StampedTransform transform;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpr1_simple_goal");

    // 构造一个导航客户对象,以便向 Navigation 系统发送导航请求
    MoveBaseClient ac("move_base", true);

    // 等待ROS的导航服务启动
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    // 以机器人本体为原点,向前走1米
    goal.target_pose.header.frame_id = "base_footprint";
    goal.target_pose.header.stamp = ros::Time::now();

    // 导航目的地坐标 ( 1.0 , 0.0 ) 前方一米处
    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.orientation.w = 1.0;

    tf::Quaternion quat;
    // 目标姿态,函数三个参数分别为滚转,俯仰和偏转角,单位为弧度
    quat.setRPY(0.0, 0.0, 3.14);
    // 将欧拉角旋转量转换成四元数表达
    transform.setRotation(quat);
    goal.target_pose.pose.orientation.x = transform.getRotation().getX();
    goal.target_pose.pose.orientation.y = transform.getRotation().getY();
    goal.target_pose.pose.orientation.z = transform.getRotation().getZ();
    goal.target_pose.pose.orientation.w = transform.getRotation().getW();

    // 发送导航坐标给 Navigation 系统
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // 等待执行结果
    ac.waitForResult();

    // 根据返回值判定执行结果
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 1 meter forward");
    else
        ROS_INFO("The base failed to move forward 1 meter for some reason");

    return 0;
}