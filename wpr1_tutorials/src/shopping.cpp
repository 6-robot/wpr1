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
/* @author Zhang Wanjie                                             */
#include <ros/ros.h>
#include <std_msgs/String.h>

#define STATE_READY     0
#define STATE_FOLLOW    1

static ros::Publisher spk_pub;
static ros::Publisher behaviors_pub;
static std_msgs::String behavior_msg;
static int nState = STATE_READY;

void KeywordCB(const std_msgs::String::ConstPtr & msg)
{
    //ROS_WARN("[KeywordCB] - %s",msg->data.c_str());
    int nFindIndex = 0;
    nFindIndex = msg->data.find("跟");
    if( nFindIndex >= 0 )
    {
        ///xfei/iat std_msgs/String -- "跟着我"
        //ROS_WARN("[KeywordCB] - 开始跟随");
        behavior_msg.data = "follow start";
        behaviors_pub.publish(behavior_msg);
    }

    nFindIndex = msg->data.find("Follow");
    if( nFindIndex >= 0 )
    {
        ///xfei/iat std_msgs/String -- "Follow me"
        //ROS_WARN("[KeywordCB] - Follow");
        behavior_msg.data = "follow start";
        behaviors_pub.publish(behavior_msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpr1_shopping");

    ros::NodeHandle n;
    ros::Subscriber sub_sr = n.subscribe("/xfei/iat", 10, KeywordCB);

    behaviors_pub = n.advertise<std_msgs::String>("wpr1_behaviors", 30);

    ROS_INFO("[main] wpr1_shopping");
    ros::Rate r(1);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
        
    }

    return 0;
}