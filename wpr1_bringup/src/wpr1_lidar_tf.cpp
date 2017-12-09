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
#include <math.h>

static double fDegToAng = 3.1415926/180;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpr1_lidar_tf");
    ROS_WARN("[wpr1_lidar_tf]");
    ros::NodeHandle n;
   
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states",10);
    sensor_msgs::JointState msg;
    std::vector<std::string> joint_name(3);
    std::vector<double> joint_pos(3);

    joint_name[0] = "laser_joint";
    joint_name[1] = "kinect_hd_joint";
    // joint_name[2] = "upperarm_to_forearm";
    // joint_name[3] = "forearm_to_palm";
    joint_pos[0] = 0.19f;
    joint_pos[1] = 1.57f;
    // joint_pos[2] = -0.7f;
    // joint_pos[3] = 0.0f;

    int nCount = 100;
    int nFlag = 0;
    ros::Rate r(30);
    while(n.ok())
    {
        msg.header.stamp = ros::Time::now();
        msg.header.seq ++;
        msg.name = joint_name;
        msg.position = joint_pos;
        joint_state_pub.publish(msg);
        
        ros::spinOnce();
        r.sleep();
    }
}