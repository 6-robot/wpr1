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
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "MPU_driver.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpr1_imu");
    ROS_INFO("[wpr1_imu]");
    ros::NodeHandle n;
    ros::Publisher out_pub = n.advertise<sensor_msgs::Imu >("imu/data_raw", 1000);

    CMPU_driver m_mpu;
    m_mpu.Open("/dev/wpr1_imu",1000000);
    
    ros::Rate r(100.0);

    while(n.ok())
    {
         m_mpu.ReadNewData();
         
        sensor_msgs::Imu imu_msg = sensor_msgs::Imu();	
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu";
        imu_msg.orientation.x = m_mpu.fQuatX;
        imu_msg.orientation.y = m_mpu.fQuatY;
        imu_msg.orientation.z = m_mpu.fQuatZ;
        imu_msg.orientation.w = m_mpu.fQuatW;

        imu_msg.angular_velocity.x = m_mpu.fGyroX;
        imu_msg.angular_velocity.y = m_mpu.fGyroY;
        imu_msg.angular_velocity.z = m_mpu.fGyroZ;

        imu_msg.linear_acceleration.x = m_mpu.fAccX;
        imu_msg.linear_acceleration.y = m_mpu.fAccY;
        imu_msg.linear_acceleration.z = m_mpu.fAccZ;

        out_pub.publish(imu_msg);

        ros::spinOnce();
        r.sleep();
    }
}