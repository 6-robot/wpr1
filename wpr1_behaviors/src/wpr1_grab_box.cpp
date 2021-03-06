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
#include <std_msgs/Int64.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_broadcaster.h>

// 抓取参数调节（单位：米）
static float obj_x_dist = 1.0f;             //抬起手臂时，机器人和物品的距离（从wpr1_agent传过来）
static float grab_y_offset = -0.05f;        //抓取前，对准物品，机器人的横向位移偏移量
static float grab_lift_offset = 0.10f;       //脊柱高度的补偿偏移量
static float obj_x_grab = 0.79f;             //机器人抓取物品时物品的距离
static float grab_gripper_value = 11500;    //抓取物品时，手爪闭合后的手爪位置

#define STEP_WAIT           0
#define STEP_OBJ_DIST       4
#define STEP_HAND_UP        5
#define STEP_FORWARD        6
#define STEP_GRAB           7
#define STEP_OBJ_UP         8
#define STEP_BACKWARD       9
#define STEP_DONE           10
static int nStep = STEP_WAIT;

static std::string pc_topic;
static ros::Publisher box_cmd_pub;
static std_msgs::String box_cmd_msg;
static ros::Publisher vel_pub;
static ros::Publisher mani_ctrl_pub;
static sensor_msgs::JointState mani_ctrl_msg;
static ros::Publisher result_pub;

void VelCmd(float inVx , float inVy, float inTz);


static std_msgs::String result_msg;

static ros::Publisher odom_ctrl_pub;
static std_msgs::String ctrl_msg;
static geometry_msgs::Pose2D pose_diff;
static geometry_msgs::Pose pose_box;

static float fBackY = 0;
static float fMoveTargetX = 0;
static float fMoveTargetY = 0;

static int nTimeDelayCounter = 0;

static float fTargetGrabX = 0.9;        //抓取时目标物品的x坐标
static float fTargetGrabY = 0.0;        //抓取时目标物品的y坐标


void BoxParamCB(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() < 2)
        return;
    obj_x_dist = msg->data[0];       //机器人和盒子的距离（从wpr1_agent传过来）
}

void GrabBoxCallback(const std_msgs::Int64::ConstPtr &msg)
{
    // 盒子编号
    int nBoxIndex = msg->data;
    ROS_WARN("[GRAB_BOX] nBoxIndex = %d " , nBoxIndex);
    switch(nBoxIndex)
    {
        case 0:
            box_cmd_msg.data = "track box 0";
            break;
        case 1:
            box_cmd_msg.data = "track box 1";
            break;
        default:
            box_cmd_msg.data = "stop track";
            break;
    }
    box_cmd_pub.publish(box_cmd_msg);

    nStep = STEP_WAIT;
}

void BoxPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    // 盒子坐标
    pose_box = *msg;
    fBackY = -pose_box.position.y;

    if( nStep == STEP_WAIT)
    {
        ROS_WARN("[BoxPoseCallback] pose_box (%.2f , %.2f , %.2f)",pose_box.position.x,pose_box.position.y,pose_box.position.z);
        nStep = STEP_OBJ_DIST;
    }
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
    int nFindIndex = msg->data.find("grab stop");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[grab_stop] ");
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
    ros::init(argc, argv, "wpr1_grab_box");
    ROS_INFO("wpr1_grab_box");

    ros::NodeHandle nh;

    box_cmd_pub = nh.advertise<std_msgs::String>("/box/command", 10);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    mani_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpr1/joint_ctrl", 30);
    result_pub = nh.advertise<std_msgs::String>("/wpr1/grab_result", 30);
    ros::Subscriber sub_grab_pose = nh.subscribe("/wpr1/grab_box", 1, GrabBoxCallback);
    ros::Subscriber sub_beh = nh.subscribe("/wpr1/behaviors", 30, BehaviorCB);
    odom_ctrl_pub = nh.advertise<std_msgs::String>("/wpr1/ctrl", 30);
    ros::Subscriber pose_diff_sub = nh.subscribe("/wpr1/pose_diff", 1, PoseDiffCallback);
    ros::Subscriber param_res_sub = nh.subscribe("/box/param", 2, BoxParamCB);
    ros::Subscriber box_pose_sub = nh.subscribe("/box/pose", 1, BoxPoseCallback);

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
    mani_ctrl_msg.position[4] = 25000;
    mani_ctrl_msg.velocity[0] = 1500;
    mani_ctrl_msg.velocity[1] = 1500;
    mani_ctrl_msg.velocity[2] = 1500;
    mani_ctrl_msg.velocity[3] = 1500;
    mani_ctrl_msg.velocity[4] = 1500;

    ros::Rate r(30);
    while(nh.ok())
    {
    
        //4、左右平移对准目标物品 
        if(nStep == STEP_OBJ_DIST)
        {
            float vx,vy;
            vx = (pose_box.position.x - obj_x_dist)/2;
            vy = (pose_box.position.y + grab_y_offset)/2;

            VelCmd(vx,vy,0);
            ROS_WARN("[STEP_OBJ_DIST] pose_box (%.2f , %.2f , %.2f) v(%.2f,%.2f)",pose_box.position.x,pose_box.position.y,pose_box.position.z,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                nTimeDelayCounter = 0;
                nStep = STEP_HAND_UP;
            }

            result_msg.data = "object x";
            result_pub.publish(result_msg);
        }

        //5、抬起手臂
        if(nStep == STEP_HAND_UP)
        {
            if(nTimeDelayCounter == 0)
            {
                float fTorsoVal = pose_box.position.z + 0.52 - 1.3 + grab_lift_offset;
                if (fTorsoVal < 0)
                {
                    fTorsoVal = 0;
                }
                if (fTorsoVal > 0.35)
                {
                    fTorsoVal = 0.35;
                }
                mani_ctrl_msg.position[0] = fTorsoVal;
                mani_ctrl_msg.position[1] = -0.52;
                mani_ctrl_msg.position[2] = -0.52;
                mani_ctrl_msg.position[3] = 0;
                mani_ctrl_msg.position[4] = 47000;
                mani_ctrl_pub.publish(mani_ctrl_msg);
                ROS_WARN("[STEP_HAND_UP] lift= %.2f  gripper= %.2f " ,fTorsoVal, mani_ctrl_msg.position[4]);
                result_msg.data = "hand up";
                result_pub.publish(result_msg);
            }
            nTimeDelayCounter ++;
            mani_ctrl_pub.publish(mani_ctrl_msg);
            VelCmd(0,0,0);
            if(nTimeDelayCounter > 5*30)
            {
                ROS_WARN("[STEP_FORWARD] pose_box (%.2f , %.2f , %.2f) " ,pose_box.position.x,pose_box.position.y,pose_box.position.z);
                nTimeDelayCounter = 0;
                nStep = STEP_FORWARD;
            }
        }

         //6、前进靠近物品
        if(nStep == STEP_FORWARD)
        {
            float vx,vy;
            vx = (pose_box.position.x - obj_x_grab)/2;
            vy = (pose_box.position.y + grab_y_offset)/2;

            VelCmd(vx,vy,0);
            //ROS_WARN("[STEP_FORWARD] pose_box (%.2f , %.2f , %.2f) v(%.2f,%.2f)",pose_box.position.x,pose_box.position.y,pose_box.position.z,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                nTimeDelayCounter = 0;
                nStep = STEP_GRAB;
            }

            result_msg.data = "forward";
            result_pub.publish(result_msg);
        }

        //7、抓取物品
        if(nStep == STEP_GRAB)
        {
            if(nTimeDelayCounter == 0)
            {
                result_msg.data = "grab";
                result_pub.publish(result_msg);
            }
            mani_ctrl_msg.position[4] = grab_gripper_value;
            mani_ctrl_pub.publish(mani_ctrl_msg);
            //ROS_WARN("[STEP_GRAB] lift= %.2f  gripper= %.2f " ,mani_ctrl_msg.position[0], mani_ctrl_msg.position[4]);

            nTimeDelayCounter++;
            VelCmd(0,0,0);
            if(nTimeDelayCounter > 3*30)
            {
                box_cmd_msg.data = "stop track";
                box_cmd_pub.publish(box_cmd_msg);

                nTimeDelayCounter = 0;
                ROS_WARN("[STEP_OBJ_UP]");
                nStep = STEP_OBJ_UP;
            }
        }

        //8、拿起物品
        if(nStep == STEP_OBJ_UP)
        {
            if(nTimeDelayCounter == 0)
            {
                mani_ctrl_msg.position[0] += 0.03;
                mani_ctrl_pub.publish(mani_ctrl_msg);
                //ROS_WARN("[MANI_CTRL] lift= %.2f  gripper= %.2f " ,mani_ctrl_msg.position[0], mani_ctrl_msg.position[4]);
                result_msg.data = "object up";
                result_pub.publish(result_msg);
                ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(ctrl_msg);
            }
            nTimeDelayCounter++;
            VelCmd(0,0,0);
            mani_ctrl_pub.publish(mani_ctrl_msg);
            if(nTimeDelayCounter > 3*30)
            {
                fMoveTargetX = -(obj_x_dist - obj_x_grab);
                fMoveTargetY = -(fBackY - grab_y_offset);
                ROS_WARN("[STEP_BACKWARD] x= %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);

                nTimeDelayCounter = 0;
                nStep = STEP_BACKWARD;
            }
        }

        //9、带着物品后退
        if(nStep == STEP_BACKWARD)
        {
            //ROS_WARN("[STEP_BACKWARD] nTimeDelayCounter = %d " ,nTimeDelayCounter);
            //nTimeDelayCounter++;
            
            float vx,vy;
            vx = (fMoveTargetX - pose_diff.x)/2;

            if(pose_diff.x > -0.3)
                vy = 0;
            else
                vy = (fMoveTargetY - pose_diff.y)/2;

            VelCmd(vx,vy,0);

            //ROS_INFO("[MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(ctrl_msg);
                nTimeDelayCounter = 0;
                ROS_WARN("[STEP_DONE]");
                nStep = STEP_DONE;
            }

            result_msg.data = "backward";
            result_pub.publish(result_msg);
        }

        //10、抓取任务完毕
        if(nStep == STEP_DONE)
        {
            if(nTimeDelayCounter < 10)
            {
                mani_ctrl_msg.position[1] = -1.57;
                mani_ctrl_msg.position[2] = -1.57;
                mani_ctrl_pub.publish(mani_ctrl_msg);
                VelCmd(0,0,0);
                nTimeDelayCounter ++;
                result_msg.data = "done";
                result_pub.publish(result_msg);
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