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


int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpr1_mani_test");
    ROS_INFO("[wpr1_mani_test]");
    ros::NodeHandle n;
    for(int i=0;i<7;i++)
    {
        fJointAngle[i] = 0;
        nJointSpeed[i] = 1500;
    }

    ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("serial_port", strSerialPort, "/dev/ttyUSB0");
    m_mani.Open(strSerialPort.c_str(),115200);

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
    joint_pos[6] = 0.94f;    //kinect_height
    joint_pos[7] = -0.17f;    //kinect_pitch

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
      
        // 测试控制
        if(nCount > 300)
        {
            nCount = 0;
            if(nFlag == 0)
            {
                ROS_WARN("[wpr1_mani_test] 0 ");
                fJointAngle[0] = 50;    //5cm
                fJointAngle[3] = -90;
                fJointAngle[4] = -40;
                fJointAngle[5] = 10;
                fJointAngle[6] = 15000;
                nFlag ++;
            }
            else
            {
                ROS_WARN("[wpr1_mani_test] 1 ");
                fJointAngle[0] = 0;
                fJointAngle[3] = 0;
                fJointAngle[4] = 0;
                fJointAngle[5] = 0;
                fJointAngle[6] = 35000;
                nFlag = 0;
            }
            m_mani.SetJoints(fJointAngle, nJointSpeed);
        }
        nCount ++;

        ros::spinOnce();
        r.sleep();
    }
}