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
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "dock_detect.h"

#define STEP_WAIT        0
#define STEP_FACETO_DOCK	1
#define STEP_ENTER_DOCK	2
#define STEP_DONE		3

static int nStep = STEP_WAIT;
static ros::Publisher result_pub;
static ros::Publisher vel_pub;
static std_msgs::String result_msg;
static CDetectDock cDock;
static int arRanges[1081];
static int nDelayCount = 0;

#define CHARGE_FACE_DIST 70
#define CHARGE_ENTER_DIST 50

void VelCmd(float inVx , float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = inVx;
    vel_cmd.linear.y = inVy;
    vel_cmd.angular.z = inTz;
    vel_pub.publish(vel_cmd);
}

double CalDirectAngle(int inFromX, int inFromY, int inToX, int inToY)
{
	double res = 0;

	int dx = inFromX - inToX;
	double dy = -(inFromY - inToY);
	if (dx == 0)
	{
		if (dy > 0)
		{
			res = 180 - 90;
		}
		else
		{
			res = 0 - 90;
		}

	}
	else
	{
		double fTan = dy / dx;
		res = atan(fTan) * 180 / 3.1415926;

		if (dx < 0)
		{
			res = res - 180;
		}

	}
	res -= 90;

	if (res < 0)
	{
		res += 360;
	}

	if (res > 360)
	{
		res -= 360;
	}

	return res;
}

float VelLimit(float inVel, float inMin, float inMax)
{
    float retVel = inVel;
    if(retVel < inMin)
    {
        retVel = inMin;
    }
    if(retVel > inMax)
    {
        retVel = inMax;
    }
    return retVel;
}

void BehaviorCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("dock start");
    if( nFindIndex >= 0 )
    {
        cDock.Reset();
        cDock.nDock_X = 0;
        cDock.nDock_Y = CHARGE_FACE_DIST;
        cDock.fDock_Angle = 0;
        cDock.SetDist(CHARGE_FACE_DIST);
        nDelayCount = 0;
        nStep = STEP_FACETO_DOCK;
        ROS_WARN("[dock_start] ");
    }

    nFindIndex = msg->data.find("dock stop");
    if( nFindIndex >= 0 )
    {
        cDock.Reset();
        cDock.nDock_X = 0;
        cDock.nDock_Y = CHARGE_FACE_DIST;
        cDock.fDock_Angle = 0;
        nStep = STEP_WAIT;
        ROS_WARN("[dock_stop] ");
    }

}

void ScanCB(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    for(int i=0;i<1081;i++)
    {
        arRanges[i] = scan->ranges[i] * 1000;
    }
    cDock.InData(arRanges);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpr1_dock_server");

    ros::NodeHandle n;
    ros::Subscriber sub_sr = n.subscribe("/wpr1/behaviors", 30, BehaviorCB);
    ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan",30,ScanCB);
    ros::Publisher joint_ctrl_pub = n.advertise<sensor_msgs::JointState>("wpr1/joint_ctrl", 30);
    result_pub = n.advertise<std_msgs::String>("/wpr1/dock_result", 30);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 30);

    bool bActive = false;
    ros::NodeHandle nh_param("~");
    nh_param.param<bool>("start", bActive, false);
    if(bActive == true)
    {
        cDock.Reset();
        cDock.nDock_X = 0;
        cDock.nDock_Y = CHARGE_FACE_DIST;
        cDock.fDock_Angle = 0;
        cDock.SetDist(CHARGE_FACE_DIST);
        nDelayCount = 0;
        nStep = STEP_FACETO_DOCK;
    }

    sensor_msgs::JointState ctrl_msg;
    ctrl_msg.name.resize(5);
    ctrl_msg.position.resize(5);
    ctrl_msg.velocity.resize(5);
    ctrl_msg.name[0] = "base_to_torso";
    ctrl_msg.name[1] = "torso_to_upperarm";
    ctrl_msg.name[2] = "upperarm_to_forearm";
    ctrl_msg.name[3] = "forearm_to_palm";
    ctrl_msg.name[4] = "gripper";
    ctrl_msg.position[0] = 0;
    ctrl_msg.position[1] = -1.57;
    ctrl_msg.position[2] = -0.7;
    ctrl_msg.position[3] = 0;
    ctrl_msg.position[4] = 15000;
    ctrl_msg.velocity[0] = 1500;
    ctrl_msg.velocity[1] = 1500;
    ctrl_msg.velocity[2] = 1500;
    ctrl_msg.velocity[3] = 1500;
    ctrl_msg.velocity[4] = 1500;

    ros::Rate r(10);
    ros::spinOnce();
    r.sleep();
    
    while(ros::ok())
    {
      
        if(nStep == STEP_FACETO_DOCK)
        {
            if(nDelayCount == 0)
            {
                result_msg.data = "face to dock";
                result_pub.publish(result_msg);
                ROS_WARN("[wpr1_dock_server] STEP_FACETO_DOCK");
                nDelayCount ++;
            }
            if (cDock.bFindDock == true)
            {
                VelCmd(0, 0, 0);
            }
            else
            {
                double dAngle = CalDirectAngle(0, 0, cDock.nFace_x, cDock.nFace_y);
                int nDist = cDock.GetDistToPoint(cDock.nFace_x, cDock.nFace_y);
                //ROS_WARN("[dock] dst=( %d , %d ) ang= %.2f dist= %d",cDock.nFace_x, cDock.nFace_y,dAngle,nDist);
                if (nDist == -1)
                {
                    int nDock_Angle = cDock.fDock_Angle;
                    float fVelTurn = nDock_Angle * -(3.14/180) * 0.5;
                    fVelTurn = VelLimit(fVelTurn,-0.3,0.3);
                    VelCmd(0, 0, fVelTurn);
                }
                else
                {
                    
                    int nDock_Angle = cDock.fDock_Angle;
                    while (nDock_Angle < -180)
                    {
                        nDock_Angle += 360;
                    }
                    while (nDock_Angle > 180)
                    {
                        nDock_Angle -= 360;
                    }
                    float fVel_Y = cDock.nFace_x * -0.005;
                    fVel_Y = VelLimit(fVel_Y,-0.1,0.1);
                    float fVel_X = cDock.nFace_y * 0.005;
                    fVel_X = VelLimit(fVel_X,-0.1,0.1);
                    float fVelTurn = nDock_Angle * -(3.14/180) * 0.5;
                    fVelTurn = VelLimit(fVelTurn,-0.3,0.3);
                    //ROS_WARN("[dock] lx=%.2f ly=%.2f az=%.2f",fVel_X, fVel_Y, fVelTurn);
                    VelCmd(fVel_X, fVel_Y, fVelTurn);

                    if (nDist < 10 && abs(nDock_Angle) < 5)
                    {
                        cDock.SetDist(CHARGE_ENTER_DIST);
                        nDelayCount = 0;
                        nStep = STEP_ENTER_DOCK;
                    }
                }
            }
        }

        if(nStep == STEP_ENTER_DOCK)
        {
            if(nDelayCount == 0)
            {
                result_msg.data = "enter dock";
                result_pub.publish(result_msg);
                ROS_WARN("[wpr1_dock_server] STEP_ENTER_DOCK");
                nDelayCount ++;
            }
            ctrl_msg.position[0] = 0.05;
            ctrl_msg.position[1] = -1.57;
            ctrl_msg.position[2] = -0.7;
            joint_ctrl_pub.publish(ctrl_msg);
            if (cDock.bFindDock == true)
            {
                VelCmd(0, 0, 0);
            }
            else
            {
                double dAngle = CalDirectAngle(0, 0, cDock.nDock_X, cDock.nDock_Y);
                int nDist = cDock.GetDistToPoint(cDock.nDock_X, cDock.nDock_Y);
                int nDock_Angle = cDock.fDock_Angle;
                while (nDock_Angle < -180)
                {
                    nDock_Angle += 360;
                }
                while (nDock_Angle > 180)
                {
                    nDock_Angle -= 360;
                }
                float fVel_Y = cDock.nDock_X * -0.0005;
                fVel_Y = VelLimit(fVel_Y,-0.05,0.05);
                float fVel_X = cDock.nDock_Y * 0.002;
                fVel_X = VelLimit(fVel_X,-0.2,0.2);
                float fVelTurn = nDock_Angle * -(3.14/180) * 0.5;
                fVelTurn = VelLimit(fVelTurn,-0.3,0.3);
                //ROS_WARN("[dock] lx=%.2f ly=%.2f az=%.2f",fVel_X, fVel_Y, fVelTurn);
                VelCmd(fVel_X, fVel_Y, fVelTurn);

                if (nDist < 23)
                {
                    VelCmd(0, 0, 0);
                    nDelayCount = 0;
                    nStep = STEP_DONE;
                }
            }

        }
        if(nStep == STEP_DONE)
        {
            if(nDelayCount == 0)
            {
                result_msg.data = "done";
                result_pub.publish(result_msg);
                ROS_WARN("[wpr1_dock_server] STEP_DONE");
                nDelayCount ++;
            }
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}