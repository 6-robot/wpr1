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
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

// 放置参数调节（单位：米）
static float place_dist = 0.9;               //机器人放置前和目标点距离
static float place_y_offset = 0.0f;          //机器人的横向位移补偿量
static float place_lift_offset = 0.0f;       //手臂抬起高度的补偿量
static float place_forward_offset = 0.0f;    //手臂抬起后，机器人向前移动的位移补偿量

static int grab_gripper_pos = 1000;       //加持物品时，手爪闭合的位置
static int place_gripper_pos = 47000;     //放置物品时，手爪松开后的位置

#define STEP_WAIT           0
#define STEP_PLACE_DIST     1
#define STEP_FORWARD        2
#define STEP_RELEASE        3
#define STEP_BACKWARD       4
#define STEP_Y_BACK         5
#define STEP_DONE           6
static int nStep = STEP_WAIT;

static std::string pc_topic;
static ros::Publisher vel_pub;
static ros::Publisher mani_ctrl_pub;
static sensor_msgs::JointState mani_ctrl_msg;
static ros::Publisher result_pub;
static std_msgs::String result_msg;
static ros::Publisher odom_ctrl_pub;
static std_msgs::String odom_ctrl_msg;
static geometry_msgs::Pose2D pose_diff;

void VelCmd(float inVx , float inVy, float inTz);

static float fPlaceX = 0;
static float fPlaceY = 0;
static float fPlaceZ = 0;
static float fMoveTargetX = 0;
static float fMoveTargetY = 0;

static int nTimeDelayCounter = 0;

void PlaceActionCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // 放置物品的坐标
    fPlaceX = msg->position.x;
    fPlaceY = msg->position.y;
    fPlaceZ = msg->position.z;
    ROS_WARN("[Place] x = %.2f y= %.2f ,z= %.2f " ,fPlaceX, fPlaceY, fPlaceZ);
    odom_ctrl_msg.data = "pose_diff reset";
    odom_ctrl_pub.publish(odom_ctrl_msg);

    // ajudge the dist to place
    fMoveTargetX = fPlaceX - place_dist;
    fMoveTargetY = fPlaceY + place_y_offset;
    ROS_WARN("[MOVE_TARGET] x = %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);
    nTimeDelayCounter = 0;
    nStep = STEP_PLACE_DIST;
}

void PoseDiffCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    pose_diff.x = msg->x;
    pose_diff.y = msg->y;
    pose_diff.theta = msg->theta;
}

void VelCmd(float inVx , float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = inVx;
    vel_cmd.linear.y = inVy;
    vel_cmd.angular.z = inTz;
    vel_pub.publish(vel_cmd);
}

void BehaviorCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = msg->data.find("place stop");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[place stop] ");
        nStep = STEP_WAIT;
        geometry_msgs::Twist vel_cmd;
        vel_cmd.linear.x = 0;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = 0;
        vel_pub.publish(vel_cmd);
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpr1_place_box");
    ROS_INFO("wpr1_place_box");

    ros::NodeHandle nh;

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    mani_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpr1/joint_ctrl", 30);
    result_pub = nh.advertise<std_msgs::String>("/wpr1/place_result", 30);

    ros::Subscriber sub_grab_pose = nh.subscribe("/wpr1/place_action", 1, PlaceActionCallback);
    ros::Subscriber sub_beh = nh.subscribe("/wpr1/behaviors", 30, BehaviorCB);
    odom_ctrl_pub = nh.advertise<std_msgs::String>("/wpr1/ctrl", 30);
    ros::Subscriber pose_diff_sub = nh.subscribe("/wpr1/pose_diff", 1, PoseDiffCallback);

    mani_ctrl_msg.name.resize(5);
    mani_ctrl_msg.position.resize(5);
    mani_ctrl_msg.velocity.resize(5);
    mani_ctrl_msg.name[0] = "base_to_torso";
    mani_ctrl_msg.name[1] = "torso_to_upperarm";
    mani_ctrl_msg.name[2] = "upperarm_to_forearm";
    mani_ctrl_msg.name[3] = "forearm_to_palm";
    mani_ctrl_msg.name[4] = "gripper";
    mani_ctrl_msg.position[0] = 0;
    mani_ctrl_msg.position[1] = -1.57;
    mani_ctrl_msg.position[2] = -0.9;
    mani_ctrl_msg.position[3] = 0;
    mani_ctrl_msg.position[4] = grab_gripper_pos;
    mani_ctrl_msg.velocity[0] = 1500;
    mani_ctrl_msg.velocity[1] = 1500;
    mani_ctrl_msg.velocity[2] = 1500;
    mani_ctrl_msg.velocity[3] = 1500;
    mani_ctrl_msg.velocity[4] = 1500;

    ros::Rate r(30);
    while(nh.ok())
    {
        //1、左右平移对准放置目标点，同时抬起手臂 
        if(nStep == STEP_PLACE_DIST)
        {
            float vx,vy;
            vx = (fMoveTargetX - pose_diff.x)/2;
            vy = (fMoveTargetY - pose_diff.y)/2;
            //ROS_INFO("[MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);
            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                if(nTimeDelayCounter > 8*30)
                {
                    odom_ctrl_msg.data = "pose_diff reset";
                    odom_ctrl_pub.publish(odom_ctrl_msg);
                    fMoveTargetX = place_dist - 0.60 + place_forward_offset;
                    fMoveTargetY = 0;
                    nTimeDelayCounter = 0;
                    nStep = STEP_FORWARD;
                    continue;
                }
            }
            else
            {
                VelCmd(vx,vy,0);
            }

            mani_ctrl_msg.position[0] = fPlaceZ + 0.57 - 1.3 + place_lift_offset;
            mani_ctrl_msg.position[1] = -0.52;
            mani_ctrl_msg.position[2] = -0.52;
            mani_ctrl_msg.position[3] = 0;
            mani_ctrl_msg.position[4] = grab_gripper_pos;
            mani_ctrl_pub.publish(mani_ctrl_msg);
            nTimeDelayCounter ++;

            result_msg.data = "dist to place";
            result_pub.publish(result_msg);
        }

        //2、前进靠近放置点
        if(nStep == STEP_FORWARD)
        {
            float vx,vy;
            vx = (fMoveTargetX - pose_diff.x)/2;
            vy = (fMoveTargetY - pose_diff.y)/2;

            VelCmd(vx,vy,0);
            //ROS_INFO("[STEP_FORWARD] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                odom_ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(odom_ctrl_msg);
                nTimeDelayCounter = 0;
                //ROS_WARN("[STEP_RELEASE] grab_gripper_value = %.2f",place_gripper_value);
                nStep = STEP_RELEASE;
            }

            result_msg.data = "forward";
            result_pub.publish(result_msg);
        }

        //3、释放物品
        if(nStep == STEP_RELEASE)
        {
            if(nTimeDelayCounter == 0)
            {
                mani_ctrl_msg.position[0] -= 0.05;
                result_msg.data = "release";
                result_pub.publish(result_msg);
            }

            mani_ctrl_msg.position[4] = place_gripper_pos;      //释放物品
            mani_ctrl_pub.publish(mani_ctrl_msg);
            //ROS_WARN("[STEP_RELEASE] lift= %.2f  gripper= %.2f " ,mani_ctrl_msg.position[0], mani_ctrl_msg.position[4]);

            nTimeDelayCounter++;
            VelCmd(0,0,0);
            if(nTimeDelayCounter > 5*30)
            {
                nTimeDelayCounter = 0;
                fMoveTargetX = -(fPlaceX - 0.60 + place_forward_offset);
                //fMoveTargetY = 0;
                fMoveTargetY = -(fPlaceY + place_y_offset);
                ROS_WARN("[STEP_BACKWARD] x= %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);
                nStep = STEP_BACKWARD;
            }
        }

        //4、后退
        if(nStep == STEP_BACKWARD)
        {
            //ROS_WARN("[STEP_BACKWARD] nTimeDelayCounter = %d " ,nTimeDelayCounter);
            //nTimeDelayCounter++;
            float vx,vy;
            vx = (fMoveTargetX - pose_diff.x)/2;
            if(fabs(fMoveTargetX - pose_diff.x) > 0.1) //距离小于0.25(0.9-0.65）时就已经安全了
            {
                vy = 0;
            }
            else
            {
                vy = (fMoveTargetY - pose_diff.y)/2;
                mani_ctrl_msg.position[0] = 0.0;
                mani_ctrl_pub.publish(mani_ctrl_msg);
                nTimeDelayCounter ++;
            }
            VelCmd(vx,vy,0);
            //ROS_INFO("[MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);
            if(fabs(vx) < 0.01 && fabs(vy) < 0.01 && nTimeDelayCounter > 8*30)
            {
                VelCmd(0,0,0);
                odom_ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(odom_ctrl_msg);
                nTimeDelayCounter = 0;
                ROS_WARN("[STEP_DONE]");
                nStep = STEP_DONE;
                // odom_ctrl_msg.data = "pose_diff reset";
                // odom_ctrl_pub.publish(odom_ctrl_msg);
                // nTimeDelayCounter = 0;
                // fMoveTargetX = 0;
                // fMoveTargetY = -(fPlaceY + place_y_offset);
                // ROS_WARN("[STEP_Y_BACK] x= %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);
                // nStep = STEP_Y_BACK;
            }

            result_msg.data = "backward";
            result_pub.publish(result_msg);
        }

        //5、横向归位
        // if(nStep == STEP_Y_BACK)
        // {
        //     //ROS_WARN("[STEP_Y_BACK] nTimeDelayCounter = %d " ,nTimeDelayCounter);
        //     //nTimeDelayCounter++;
        //     float vx,vy;
        //     vx = (fMoveTargetX - pose_diff.x)/2;
        //     vy = (fMoveTargetY - pose_diff.y)/2;
        //     VelCmd(vx,vy,0);
        //     //ROS_INFO("[MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);
        //     if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
        //     {
        //         VelCmd(0,0,0);
        //         odom_ctrl_msg.data = "pose_diff reset";
        //         odom_ctrl_pub.publish(odom_ctrl_msg);
        //         nTimeDelayCounter = 0;
        //         ROS_WARN("[STEP_DONE]");
        //         nStep = STEP_DONE;
        //     }
        //     mani_ctrl_msg.position[0] = 0.5;
        //     mani_ctrl_pub.publish(mani_ctrl_msg);

        //     result_msg.data = "backward";
        //     result_pub.publish(result_msg);
        // }

        //6、放置任务完毕
        if(nStep == STEP_DONE)
        {
            if(nTimeDelayCounter < 30)
            {
                VelCmd(0,0,0);
                nTimeDelayCounter ++;
                result_msg.data = "done";
                result_pub.publish(result_msg);

                mani_ctrl_msg.position[0] = 0;
                mani_ctrl_msg.position[1] = -1.57;
                mani_ctrl_msg.position[2] = -0.9;
                mani_ctrl_msg.position[3] = 0;
                mani_ctrl_pub.publish(mani_ctrl_msg);

            }
            else
            {
                nStep = STEP_WAIT;
            }
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}