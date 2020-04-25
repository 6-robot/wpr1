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
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include "Mani_driver.h"
#include <math.h>

static double fDegToAng = 3.1415926/180;
static double fAngToDeg = 180/3.1415926;
static double fJointAngle[7];
static int nJointSpeed[7];

static CMani_driver m_mani;

void JointCtrlCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    int nNumJoint = msg->position.size();
    for(int i=0;i<nNumJoint;i++)
    {
        if(msg->name[i] == "base_to_torso")
        {
            //脊柱升降
            fJointAngle[0] = msg->position[i] * 1000;
            nJointSpeed[0] = msg->velocity[i];
        }
        if(msg->name[i] == "torso_to_upperarm")
        {
            //手臂根关节
            fJointAngle[3] = msg->position[i] * fAngToDeg;
            nJointSpeed[3] = msg->velocity[i];
        }
        if(msg->name[i] == "upperarm_to_forearm")
        {
            //手臂肘关节
            fJointAngle[4] = msg->position[i] * fAngToDeg;
            nJointSpeed[4] = msg->velocity[i];
        }
        if(msg->name[i] == "forearm_to_palm")
        {
            //手腕关节
            fJointAngle[5] = msg->position[i] * fAngToDeg;
            nJointSpeed[5] = msg->velocity[i];
        }
        if(msg->name[i] == "gripper")
        {
             //手爪
            fJointAngle[6] = msg->position[i];
            nJointSpeed[6] = msg->velocity[i];
        }
        //ROS_INFO("[wpr1_mani_cb] %d - %s = %.2f", i, msg->name[i].c_str(),msg->position[i]);
    }
    
    // //脊柱升降
    // fJointAngle[0] = msg->position[0] * 1000;
    // nJointSpeed[0] = msg->velocity[0];
    // //手臂根关节
    // fJointAngle[3] = msg->position[1] * fAngToDeg;
    // nJointSpeed[3] = msg->velocity[1];
    // //手臂肘关节
    // fJointAngle[4] = msg->position[2] * fAngToDeg;
    // nJointSpeed[4] = msg->velocity[2];
    // //手腕关节
    // fJointAngle[5] = msg->position[3] * fAngToDeg;
    // nJointSpeed[5] = msg->velocity[3];
    // //手爪
    // fJointAngle[6] = msg->position[4];
    // nJointSpeed[6] = msg->velocity[4];

    m_mani.SetJoints(fJointAngle, nJointSpeed);
    //ROS_INFO("[wpr1_mani_SetJoints] %.0f %.0f %.0f %.0f %.0f", fJointAngle[0],fJointAngle[1],fJointAngle[2],fJointAngle[3],fJointAngle[4]);
    //ROS_INFO("[wpr1_mani_SetVel] %d %d %d %d %d", nJointSpeed[0],nJointSpeed[1],nJointSpeed[2],nJointSpeed[3],nJointSpeed[4]);
}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpr1_manipulator");
    ROS_INFO("[wpr1_manipulator]");
    ros::NodeHandle n;
    for(int i=0;i<7;i++)
    {
        fJointAngle[i] = 0;
        nJointSpeed[i] = 1500;
    }
    fJointAngle[0] = 150;
    fJointAngle[3] = -1.57 * fAngToDeg;
    fJointAngle[4] = -0.7 * fAngToDeg;
    fJointAngle[5] = 0 * fAngToDeg;
    fJointAngle[6] = 25000;

    ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("serial_port", strSerialPort, "/dev/wpr1_mani");
    m_mani.Open(strSerialPort.c_str(),115200);

    ros::Subscriber joint_ctrl_sub = n.subscribe("/wpr1/joint_ctrl",30,&JointCtrlCallback);
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states",100);
    sensor_msgs::JointState msg;
    std::vector<std::string> joint_name(8);
    std::vector<double> joint_pos(8);

    joint_name[0] = "base_to_torso";
    joint_name[1] = "torso_to_upperarm";
    joint_name[2] = "upperarm_to_forearm";
    joint_name[3] = "forearm_to_palm";
    joint_name[4] = "mani_palm_finger";
    joint_name[5] = "mani_palm_right_finger";
    joint_name[6] = "kinect_height";
    joint_name[7] = "kinect_pitch";
    joint_pos[0] = 0.0f;
    joint_pos[1] = 1.57f;
    joint_pos[2] = -0.7f;
    joint_pos[3] = 0.0f;
    joint_pos[4] = 0.0f;
    joint_pos[5] = 0.0f;
    joint_pos[6] = 1.05f;    //kinect_height
    joint_pos[7] = -0.24f;    //kinect_pitch

    int nCount = 100;
    int nFlag = 0;
    ros::Rate r(30);
    while(n.ok())
    {
        m_mani.ReadNewData();
        msg.header.stamp = ros::Time::now();
        msg.header.seq ++;
        msg.name = joint_name;
        msg.position = joint_pos;
        joint_state_pub.publish(msg);
        
        //////////////////////////
        double fTmp = 0;
        //脊柱升降
        fTmp = m_mani.nRecvJointPos[0];
        joint_pos[0] = fTmp*0.001;//脊柱升降

        //手臂根关节(值增大手臂往下)
        fTmp = m_mani.nRecvJointPos[3];
        fTmp *= 0.01;
        joint_pos[1] = fTmp*fDegToAng*-1;

        //手臂肘关节(值增大手臂往下)
        fTmp = m_mani.nRecvJointPos[4];
        fTmp *= 0.01;
        joint_pos[2] = fTmp*fDegToAng*1;

        //手腕关节（值增大顺时针转）
        fTmp = m_mani.nRecvJointPos[5];
        fTmp *= 0.01;
        joint_pos[3] = fTmp*fDegToAng*1;

        //手爪
        fTmp = 50000 - m_mani.nRecvJointPos[6];
        if(fTmp < 0) fTmp = 0;
        fTmp *= -0.5/50000;
        fTmp += 0.5;
        joint_pos[4] = fTmp;
        joint_pos[5] = fTmp;

        ////////////////////////////////////////////////////////////
        // printf("[关节电机位置] ");
		// for (int i = 0; i < 7; i++)
		// {
		// 	printf("关节%d = %.6d ", i + 1, m_mani.nRecvJointPos[i]);
		// }
		// printf("\n");
        ///////////////////////////////////////////////////////////
        // 测试控制用
        // if(nCount > 300)
        // {
        //     nCount = 0;
        //     if(nFlag == 0)
        //     {
        //         ROS_WARN("[wpr1_mani] 0 ");
        //         fJointAngle[0] = 50;    //5cm
        //         fJointAngle[3] = -90;
        //         fJointAngle[4] = -40;
        //         fJointAngle[5] = 0;
        //         fJointAngle[6] = 7000;
        //         nFlag ++;
        //     }
        //     else
        //     {
        //         ROS_WARN("[wpr1_mani] 1 ");
        //         fJointAngle[0] = 0;
        //         fJointAngle[3] = 0;
        //         fJointAngle[4] = 0;
        //         fJointAngle[5] = 0;
        //         fJointAngle[6] = 25000;
        //         nFlag = 0;
        //     }
        //     m_mani.SetJoints(fJointAngle, nJointSpeed);
        // }
        ///////////////////////////////////////////////////////////
        nCount ++;

        ros::spinOnce();
        r.sleep();
    }
}
