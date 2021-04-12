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

#include "action_manager.h"
#include "xfyun_waterplus/IATSwitch.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <waterplus_map_tools/GetWaypointByName.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
static ros::Publisher spk_pub;
static ros::Publisher vel_pub;
static string strToSpeak = "";
static string strKeyWord = "";
static ros::ServiceClient clientIAT;
static xfyun_waterplus::IATSwitch srvIAT;
static ros::ServiceClient cliGetWPName;
static waterplus_map_tools::GetWaypointByName srvName;
static ros::Publisher behaviors_pub;
static std_msgs::String behavior_msg;

CActionManager::CActionManager()
{
    nCurActIndex = 0;
    nCurActCode = -1;
    strListen = "";
    bGrabDone = false;
	bPassDone = false;
	bDockDone = false;
}

CActionManager::~CActionManager()
{

}

void CActionManager::Init()
{
    ros::NodeHandle n;
    cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    spk_pub = n.advertise<std_msgs::String>("/xfyun/tts", 20);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    clientIAT = n.serviceClient<xfyun_waterplus::IATSwitch>("xfyun_waterplus/IATSwitch");
    behaviors_pub = n.advertise<std_msgs::String>("/wpr1/behaviors", 30);
    grab_result_sub = n.subscribe<std_msgs::String>("/wpr1/grab_result",30,&CActionManager::GrabResultCallback,this);
    pass_result_sub = n.subscribe<std_msgs::String>("/wpr1/pass_result",30,&CActionManager::PassResultCallback,this);
    dock_result_sub = n.subscribe<std_msgs::String>("/wpr1/dock_result",30,&CActionManager::DockResultCallback,this);
}

static void FollowSwitch(bool inActive, float inDist)
{
    if(inActive == true)
    {
        behavior_msg.data = "follow start";
        behaviors_pub.publish(behavior_msg);
    }
    else
    {
        behavior_msg.data = "follow stop";
        behaviors_pub.publish(behavior_msg);
    }
}

static void GrabSwitch(bool inActive)
{
    if(inActive == true)
    {
        behavior_msg.data = "grab start";
        behaviors_pub.publish(behavior_msg);
    }
    else
    {
        behavior_msg.data = "grab stop";
        behaviors_pub.publish(behavior_msg);
    }
}

static void PassSwitch(bool inActive)
{
    if(inActive == true)
    {
        behavior_msg.data = "pass start";
        behaviors_pub.publish(behavior_msg);
    }
    else
    {
        behavior_msg.data = "pass stop";
        behaviors_pub.publish(behavior_msg);
    }
}

static void DockSwitch(bool inActive)
{
    if(inActive == true)
    {
        behavior_msg.data = "dock start";
        behaviors_pub.publish(behavior_msg);
    }
    else
    {
        behavior_msg.data = "dock stop";
        behaviors_pub.publish(behavior_msg);
    }
}

static int nLastActCode = -1;
static geometry_msgs::Twist vel_cmd;
bool CActionManager::Main()
{
    int nNumOfAct = arAct.size();
    if(nCurActIndex >= nNumOfAct)
    {
        return false;
    }
    int nKeyWord = -1;
    nCurActCode = arAct[nCurActIndex].nAct;
    switch (nCurActCode)
	{
	case ACT_GOTO:
		if (nLastActCode != ACT_GOTO)
		{
            FollowSwitch(false, 0);
			string strGoto = arAct[nCurActIndex].strTarget;
            printf("[ActMgr] %d - Goto %s",nCurActIndex,strGoto.c_str());
            srvName.request.name = strGoto;
            if (cliGetWPName.call(srvName))
            {
                std::string name = srvName.response.name;
                float x = srvName.response.pose.position.x;
                float y = srvName.response.pose.position.y;
                ROS_INFO("Get_wp_name: name = %s (%.2f,%.2f)", strGoto.c_str(),x,y);

                MoveBaseClient ac("move_base", true);
                if(!ac.waitForServer(ros::Duration(5.0)))
                {
                    ROS_INFO("The move_base action server is no running. action abort...");
                }
                else
                {
                    move_base_msgs::MoveBaseGoal goal;
                    goal.target_pose.header.frame_id = "map";
                    goal.target_pose.header.stamp = ros::Time::now();
                    goal.target_pose.pose = srvName.response.pose;
                    ac.sendGoal(goal);
                    ac.waitForResult();
                    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                        ROS_INFO("Arrived at %s!",strGoto.c_str());
                    else
                        ROS_INFO("Failed to get to %s ...",strGoto.c_str() );
                }
                
            }
            else
            {
                ROS_ERROR("Failed to call service GetWaypointByName");
            }
            nCurActIndex ++;
        }
		break;

	case ACT_FIND_OBJ:
		if (nLastActCode != ACT_FIND_OBJ)
		{
            printf("[ActMgr] %d - Find %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            nCurActIndex ++;
		}
		break;

	case ACT_GRAB:
		if (nLastActCode != ACT_GRAB)
		{
            printf("[ActMgr] %d - Grab %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            bGrabDone = false;
            GrabSwitch(true);
		}
        if(bGrabDone == true)
        {
            printf("[ActMgr] %d - Grab %s done!\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            GrabSwitch(false);
            nCurActIndex ++;
        }
		break;

	case ACT_PASS:
		if (nLastActCode != ACT_PASS)
		{
            printf("[ActMgr] %d - Pass %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            bPassDone = false;
            PassSwitch(true);
		}
        if(bPassDone == true)
        {
            printf("[ActMgr] %d - Pass %s done!\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            PassSwitch(false);
            nCurActIndex ++;
        }
		break;

	case ACT_SPEAK:
		if (nLastActCode != ACT_SPEAK)
		{
            printf("[ActMgr] %d - Speak %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            strToSpeak = arAct[nCurActIndex].strTarget;
            std_msgs::String rosSpeak; 
            rosSpeak.data = strToSpeak;
            spk_pub.publish(rosSpeak);
            strToSpeak = "";
            printf("[ActMgr] %d - sleep %.1f\n",nCurActIndex,arAct[nCurActIndex].nDuration*1000*1000);
            usleep(arAct[nCurActIndex].nDuration*1000*1000);
            printf("[ActMgr] %d - wakeup\n",nCurActIndex);
            nCurActIndex ++;
		}
		break;

	case ACT_LISTEN:
		if (nLastActCode != ACT_LISTEN)
		{
            printf("[ActMgr] %d - Listen %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            strListen = "";
            strKeyWord = arAct[nCurActIndex].strTarget;
            int nDur = arAct[nCurActIndex].nDuration;
            if(nDur < 3)
            {
                nDur = 3;
            }
            //开始语音识别
            srvIAT.request.active = true;
            srvIAT.request.duration = nDur;
            clientIAT.call(srvIAT);
		}
        nKeyWord = strListen.find(strKeyWord);
        if(nKeyWord >= 0)
        {
            //识别完毕,关闭语音识别
            srvIAT.request.active = false;
            clientIAT.call(srvIAT);
            nCurActIndex ++;
        }
		break;

    case ACT_MOVE:
        FollowSwitch(false, 0);
        printf("[ActMgr] %d - Move ( %.2f , %.2f ) - %.2f\n",nCurActIndex,arAct[nCurActIndex].fLinear_x,arAct[nCurActIndex].fLinear_y,arAct[nCurActIndex].fAngular_z);
        vel_cmd.linear.x = arAct[nCurActIndex].fLinear_x;
        vel_cmd.linear.y = arAct[nCurActIndex].fLinear_y;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = arAct[nCurActIndex].fAngular_z;
        vel_pub.publish(vel_cmd);

        usleep(arAct[nCurActIndex].nDuration*1000*1000);
        nCurActIndex ++;
		break;

	case ACT_FOLLOW:
		if (nLastActCode != ACT_FOLLOW)
		{
            printf("[ActMgr] %d - Follow dist = %.2f \n", nCurActIndex, arAct[nCurActIndex].fFollowDist);
            FollowSwitch(true, arAct[nCurActIndex].fFollowDist);
            nCurActIndex ++;
		}
		break;

    case ACT_DOCK:
		if (nLastActCode != ACT_DOCK)
		{
            printf("[ActMgr] %d - dock start\n",nCurActIndex);
            bPassDone = false;
            PassSwitch(true);
		}
        if(bDockDone == true)
        {
            printf("[ActMgr] %d - dock done!\n",nCurActIndex);
            PassSwitch(false);
            nCurActIndex ++;
        }
		break;

	default:
		break;
	}
	nLastActCode = nCurActCode;
    ros::spinOnce();
    return true;
}

void CActionManager::Reset()
{
    strToSpeak = "";
    nCurActIndex = 0;
	nLastActCode = 0;
    arAct.clear();
}

string CActionManager::GetToSpeak()
{
    string strRet = strToSpeak;
    strToSpeak = "";
    return strRet;
}

string ActionText(stAct* inAct)
{
    string ActText = "";
    if(inAct->nAct == ACT_GOTO)
    {
        ActText = "去往地点 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_FIND_OBJ)
    {
        ActText = "搜索物品 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_GRAB)
    {
        ActText = "抓取物品 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_PASS)
    {
        ActText = "把物品递给 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_SPEAK)
    {
        ActText = "说话 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_LISTEN)
    {
        ActText = "听取关键词 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_MOVE)
    {
        ActText = "移动 ( ";
        std::ostringstream stringStream;
        stringStream << inAct->fLinear_x << " , " << inAct->fLinear_y << " ) - " << inAct->fAngular_z;
        std::string retStr = stringStream.str();
        ActText += retStr;
    }
    if(inAct->nAct == ACT_FOLLOW)
    {
        ActText = "跟随 距离为 ";
        std::ostringstream stringStream;
        stringStream << inAct->fFollowDist;
        std::string retStr = stringStream.str();
        ActText += retStr;
    }
    if(inAct->nAct == ACT_FOLLOW)
    {
        ActText = "自主充电 ";
    }
    return ActText;
}

void CActionManager::ShowActs()
{
    printf("\n*********************************************\n");
    printf("显示行为队列:\n");
    int nNumOfAct = arAct.size();
    stAct tmpAct;
    for(int i=0;i<nNumOfAct;i++)
    {
        tmpAct = arAct[i];
        string act_txt = ActionText(&tmpAct);
        printf("行为 %d : %s\n",i+1,act_txt.c_str());
    }
    printf("*********************************************\n\n");
}

void CActionManager::GrabResultCallback(const std_msgs::String::ConstPtr& res)
{
    int nFindIndex = 0;
    nFindIndex = res->data.find("done");
    if( nFindIndex >= 0 )
    {
        bGrabDone = true;
    }
}

void CActionManager::PassResultCallback(const std_msgs::String::ConstPtr& res)
{
    int nFindIndex = 0;
    nFindIndex = res->data.find("done");
    if( nFindIndex >= 0 )
    {
        bPassDone = true;
    }
}

void CActionManager::DockResultCallback(const std_msgs::String::ConstPtr& res)
{
    int nFindIndex = 0;
    nFindIndex = res->data.find("done");
    if( nFindIndex >= 0 )
    {
        bDockDone = true;
    }
}
