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
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/Image.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <sound_play/SoundRequest.h>
#include <geometry_msgs/Pose2D.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>
#include "obj_track.h"

// 抓取参数调节（单位：米）
static float fHandUpDist = 0.85;            //抬臂前，与货柜隔板之间的距离
static float fGrabOffsetY = 0.05;           //抬臂时，目标物品的y坐标（机器人对准物品的左右偏移量）
static float fGrabObjDist = 0.68;           //抓取物品时，物品和机器人的距离
static float grab_gripper_value = 25000;    //抓取物品时，手爪闭合后的手爪位置

#define STEP_WAIT           0
#define STEP_RACK_DIST      1
#define STEP_HAND_UP        2
#define STEP_FORWARD        3
#define STEP_GRAB           4
#define STEP_OBJ_UP         5
#define STEP_BACKWARD       6
#define STEP_DONE           7
static int nStep = STEP_WAIT;

static std::string pc_topic;
static ros::Publisher pc_pub;
static ros::Publisher marker_pub;
static ros::Publisher vel_pub;
static ros::Publisher mani_ctrl_pub;
static ros::Publisher result_pub;
static tf::TransformListener *tf_listener; 

void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB);
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB);
void DrawPath(float inX, float inY, float inZ);
void RemoveBoxes();
void VelCmd(float inVx , float inVy, float inTz);
void ManiCmd(float inTorso, float inShoulder, float inElbow, float inWrist, float inGripper);

static visualization_msgs::Marker line_box;
static visualization_msgs::Marker line_follow;
static visualization_msgs::Marker text_marker;

static sensor_msgs::JointState mani_ctrl_msg;
static std_msgs::String result_msg;

static ros::Publisher odom_ctrl_pub;
static std_msgs::String odom_ctrl_msg;
static geometry_msgs::Pose2D pose_diff;

static float fObjGrabX = 0;
static float fObjGrabY = 0;
static float fObjGrabZ = 0;
static float fMoveTargetX = 0;
static float fMoveTargetY = 0;
static float fRackDist = 0;
static int nTimeDelayCounter = 0;
static int nForwardCounter = 0;

typedef struct stBoxMarker
{
    float xMax;
    float xMin;
    float yMax;
    float yMin;
    float zMax;
    float zMin;
}stBoxMarker;

static stBoxMarker boxMarker;
static stBoxMarker boxPlane;
static stBoxMarker obj_to_track;

static pcl::PassThrough<pcl::PointXYZRGB> obj_pass;//设置滤波器对象
static float filter_x_min = 0;
static float filter_x_max = 2.0;
static float filter_y_min = -1.5;
static float filter_y_max = 1.5;
static float filter_z_min = 0;
static float filter_z_max = 2.0;
static pcl::search::KdTree<pcl::PointXYZRGB>::Ptr obj_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
static bool bStopCloud = true;

void ProcCloudCB(const sensor_msgs::PointCloud2 &input)
{
    //ROS_WARN("ProcCloudCB");
    if(bStopCloud == true)
        return;

    //to footprint
    sensor_msgs::PointCloud2 pc_footprint;
    bool res = tf_listener->waitForTransform("/base_footprint", input.header.frame_id, input.header.stamp, ros::Duration(5.0)); 
    if(res == false)
    {
        return;
    }
    pcl_ros::transformPointCloud("/base_footprint", input, pc_footprint, *tf_listener);

    //source cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
    pcl::fromROSMsg(pc_footprint , cloud_src);
    ROS_INFO("[ProcCloudCB]cloud_src size = %d",(int)(cloud_src.size())); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_ptr;
    cloud_source_ptr = cloud_src.makeShared(); 

    if(nStep == STEP_RACK_DIST)
    {
        // 判断一下货柜的距离
        float tMinX,tMaxY,tMinY,tMaxZ,tMinZ;
        tMaxY = fObjGrabY + 0.1;
        tMinY = fObjGrabY - 0.1;
        tMaxZ = fObjGrabZ + 0.1;
        tMinZ = fObjGrabZ - 0.5;
        tMinX = fObjGrabX;
        int nPointsNum = cloud_source_ptr->points.size();
        for(int i=0;i<nPointsNum;i++)
        {
            if(cloud_source_ptr->points[i].y > tMinY && cloud_source_ptr->points[i].y < tMaxY && cloud_source_ptr->points[i].z > tMinZ && cloud_source_ptr->points[i].z < tMaxZ)
            {
                if(cloud_source_ptr->points[i].x < tMinX)
                {
                    tMinX = cloud_source_ptr->points[i].x;
                }
            }
        }
        fRackDist = tMinX;
    }

    // 跟踪目标物体
    filter_z_min = fObjGrabZ + 0.05;
    filter_z_max = fObjGrabZ + 0.25;
    filter_x_min = fObjGrabX - 0.1;
    filter_x_max = fObjGrabX + 0.05;
    filter_y_min = fObjGrabY - 0.08;
    filter_y_max = fObjGrabY + 0.08;
    obj_pass.setInputCloud (cloud_source_ptr);
    obj_pass.setFilterFieldName ("z");
    obj_pass.setFilterLimits (filter_z_min, filter_z_max);
    obj_pass.filter (*cloud_source_ptr);
    obj_pass.setFilterFieldName ("x");
    obj_pass.setFilterLimits (filter_x_min, filter_x_max);
    obj_pass.filter (*cloud_source_ptr);
    obj_pass.setFilterFieldName ("y");
    obj_pass.setFilterLimits (filter_y_min, filter_y_max);
    obj_pass.filter (*cloud_source_ptr);
        
    // 联通域检测
    obj_tree->setInputCloud(cloud_source_ptr);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.03); // 3cm
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod (obj_tree);
    ec.setInputCloud (cloud_source_ptr);
    ec.extract (cluster_indices);

    int nObjPointNum = 0;
    stBoxMarker obj_marker;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        // 把点数最多的点云簇挑选出来作为目标物体
        if( it->indices.size() > nObjPointNum)
        {
            nObjPointNum = it->indices.size();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back (cloud_source_ptr->points[*pit]); 
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            bool bFirstPoint = true;
            for (int i=0;i<cloud_cluster->width;i++)
            {
                if(bFirstPoint == true)
                {
                    bFirstPoint = false;
                    obj_marker.xMax = obj_marker.xMin = cloud_cluster->points[i].x;
                    obj_marker.yMax = obj_marker.yMin = cloud_cluster->points[i].y;
                    obj_marker.zMax = obj_marker.zMin = cloud_cluster->points[i].z;
                }
                if(cloud_cluster->points[i].x > obj_marker.xMax) obj_marker.xMax = cloud_cluster->points[i].x;
                if(cloud_cluster->points[i].x < obj_marker.xMin) obj_marker.xMin = cloud_cluster->points[i].x;
                if(cloud_cluster->points[i].y > obj_marker.yMax) obj_marker.yMax = cloud_cluster->points[i].y;
                if(cloud_cluster->points[i].y < obj_marker.yMin) obj_marker.yMin = cloud_cluster->points[i].y;
                if(cloud_cluster->points[i].z > obj_marker.zMax) obj_marker.zMax = cloud_cluster->points[i].z;
                if(cloud_cluster->points[i].z < obj_marker.zMin) obj_marker.zMin = cloud_cluster->points[i].z;
            }
        }
    }
    RemoveBoxes();
    DrawBox(obj_marker.xMin, obj_marker.xMax, obj_marker.yMin, obj_marker.yMax, obj_marker.zMin, obj_marker.zMax, 1.0f, 0, 1.0f);

    fObjGrabX = (obj_marker.xMin + obj_marker.xMax)/2;
    fObjGrabY = (obj_marker.yMin + obj_marker.yMax)/2;
    ROS_WARN("[wpr1_grab_rack] ObjTrack ( %.2f , %.2f , %.2f )",fObjGrabX,fObjGrabY,fObjGrabZ);
}

void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB)
{
    line_box.header.frame_id = "base_footprint";
    line_box.ns = "line_box";
    line_box.action = visualization_msgs::Marker::ADD;
    line_box.id = 0;
    line_box.type = visualization_msgs::Marker::LINE_LIST;
    line_box.scale.x = 0.005;
    line_box.color.r = inR;
    line_box.color.g = inG;
    line_box.color.b = inB;
    line_box.color.a = 1.0;

    geometry_msgs::Point p;
    p.z = inMinZ;
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

    p.z = inMaxZ;
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);
    marker_pub.publish(line_box);
}

void PoseDiffCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    pose_diff.x = msg->x;
    pose_diff.y = msg->y;
    pose_diff.theta = msg->theta;
}

static int nTextNum = 2;
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB)
{
    text_marker.header.frame_id = "base_footprint";
    text_marker.ns = "line_obj";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = nTextNum;
    nTextNum ++;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = inScale;
    text_marker.color.r = inR;
    text_marker.color.g = inG;
    text_marker.color.b = inB;
    text_marker.color.a = 1.0;

    text_marker.pose.position.x = inX;
    text_marker.pose.position.y = inY;
    text_marker.pose.position.z = inZ;
    
    text_marker.pose.orientation=tf::createQuaternionMsgFromYaw(1.0);

    text_marker.text = inText;

    marker_pub.publish(text_marker);
}

void RemoveBoxes()
{
    line_box.action = 3;
    line_box.points.clear();
    marker_pub.publish(line_box);
    line_follow.action = 3;
    line_follow.points.clear();
    marker_pub.publish(line_follow);
    text_marker.action = 3;
    marker_pub.publish(text_marker);
}

void VelCmd(float inVx , float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = inVx;
    vel_cmd.linear.y = inVy;
    vel_cmd.angular.z = inTz;
    vel_pub.publish(vel_cmd);
}


void ManiCmd(float inTorso, float inShoulder, float inElbow, float inWrist, float inGripper)
{
    mani_ctrl_msg.position[0] = inTorso;
    mani_ctrl_msg.position[1] = inShoulder;
    mani_ctrl_msg.position[2] = inElbow;
    mani_ctrl_msg.position[3] = inWrist;
    mani_ctrl_msg.position[4] = inGripper;
    mani_ctrl_pub.publish(mani_ctrl_msg);
}

void GrabRackCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // 目标物品的坐标
    fObjGrabX = msg->position.x;
    fObjGrabY = msg->position.y;
    fObjGrabZ = msg->position.z;
    ROS_WARN("[GrabRackCallback] x = %.2f y= %.2f ,z= %.2f " ,fObjGrabX, fObjGrabY, fObjGrabZ);
    odom_ctrl_msg.data = "pose_diff reset";
    odom_ctrl_pub.publish(odom_ctrl_msg);

    nStep = STEP_RACK_DIST;
    bStopCloud = false;
}

void BehaviorCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("grab stop");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[grab_stop] ");
        nStep = STEP_WAIT;
        bStopCloud = true;
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
    ros::init(argc, argv, "wpr1_grab_rack");
    ROS_INFO("wpr1_grab_rack");
    tf_listener = new tf::TransformListener(); 

    ros::NodeHandle nh_param("~");
    nh_param.param<std::string>("topic", pc_topic, "/kinect2/sd/points");
   

    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe(pc_topic, 10 , ProcCloudCB);

    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("obj_pointcloud",1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("obj_marker", 10);

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    mani_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpr1/joint_ctrl", 30);
    result_pub = nh.advertise<std_msgs::String>("/wpr1/grab_result", 30);

    ros::Subscriber sub_grab_pose = nh.subscribe("/wpr1/grab_rack", 1, GrabRackCallback);
    ros::Subscriber sub_sr = nh.subscribe("/wpr1/behaviors", 30, BehaviorCB);
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
    mani_ctrl_msg.position[2] = -0.7;
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
        //1、运动对准目标物且与货柜保持特定距离
        if(nStep == STEP_RACK_DIST)
        {
            float vx,vy;
            vx = (fRackDist - fHandUpDist)/2;
            vy = (fObjGrabY - fGrabOffsetY)/2;

            VelCmd(vx,vy,0);
            //ROS_INFO("[MOVE] T(%.2f %.2f)  od(%.2f , %.2f) v(%.2f,%.2f)" ,fMoveTargetX, fMoveTargetY, pose_diff.x ,pose_diff.y,vx,vy);

            if(fabs(vx) < 0.01 && fabs(vy) < 0.01)
            {
                VelCmd(0,0,0);
                nTimeDelayCounter = 0;
                bStopCloud = true;
                nStep = STEP_HAND_UP;
                //ROS_INFO("STEP_OBJ_DIST -> STEP_HAND_UP");
            }

            result_msg.data = "rack dist";
            result_pub.publish(result_msg);
        }

        //2、抬起手臂
        if(nStep == STEP_HAND_UP)
        {
            if(nTimeDelayCounter == 0)
            {
                float fTorsoVal = fObjGrabZ + 0.54 - 1.3;
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
                fMoveTargetX = fObjGrabX -0.60 ;
                fMoveTargetY = 0;
                ROS_WARN("[STEP_FORWARD] x = %.2f y= %.2f " ,fMoveTargetX, fMoveTargetY);
                nTimeDelayCounter = 0;
                nForwardCounter = 0;
                bStopCloud = false;
                //ctrl_msg.data = "pose_diff reset";
                //odom_ctrl_pub.publish(ctrl_msg);
                nStep = STEP_FORWARD;
            }
        }


         //3、前进靠近物品
        if(nStep == STEP_FORWARD)
        {
            float vx,vy;
            vx = (fObjGrabX - fGrabObjDist)/2;
            vy = (fObjGrabY - fGrabOffsetY)/1;

            VelCmd(vx,vy,0);
            result_msg.data = "forward";
            result_pub.publish(result_msg);

            if(fabs(fObjGrabX - fGrabObjDist) < 0.01)
            {
                nForwardCounter = 0;
                nTimeDelayCounter = 0;
                bStopCloud = true;
                VelCmd(0,0,0);
                nStep = STEP_GRAB;
            }
        }

        //4、抓取物品
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
                nTimeDelayCounter = 0;
                ROS_WARN("[STEP_OBJ_UP]");
                nStep = STEP_OBJ_UP;
            }
        }

        //5、拿起物品
        if(nStep == STEP_OBJ_UP)
        {
            if(nTimeDelayCounter == 0)
            {
                mani_ctrl_msg.position[0] += 0.03;
                mani_ctrl_pub.publish(mani_ctrl_msg);
                //ROS_WARN("[MANI_CTRL] lift= %.2f  gripper= %.2f " ,mani_ctrl_msg.position[0], mani_ctrl_msg.position[4]);
                result_msg.data = "object up";
                result_pub.publish(result_msg);
            }
            nTimeDelayCounter++;
            VelCmd(0,0,0);
            mani_ctrl_pub.publish(mani_ctrl_msg);
            if(nTimeDelayCounter > 3*30)
            {
                ROS_WARN("[STEP_BACKWARD] x= %.2f y= %.2f " ,pose_diff.x, pose_diff.y);

                RemoveBoxes();
                nTimeDelayCounter = 0;
                nStep = STEP_BACKWARD;
                continue;
            }
        }

        //6、带着物品后退
        if(nStep == STEP_BACKWARD)
        {
            //ROS_WARN("[STEP_BACKWARD] nTimeDelayCounter = %d " ,nTimeDelayCounter);
            //nTimeDelayCounter++;
            
            float vx,vy;
            vx = (- pose_diff.x)/2;
            if(vx < -0.1)
            {
                vx = -0.1;
            }
            vy = (- pose_diff.y)/2;

            VelCmd(vx,vy,0);

            //ROS_INFO("[STEP_BACKWARD] od(%.2f , %.2f) v(%.2f,%.2f)" ,pose_diff.x ,pose_diff.y,vx,vy);

            if(fabs(pose_diff.x) < 0.02 && fabs(vy) < 0.02)
            {
                VelCmd(0,0,0);
                odom_ctrl_msg.data = "pose_diff reset";
                odom_ctrl_pub.publish(odom_ctrl_msg);

                mani_ctrl_msg.position[1] = -1.57;
                mani_ctrl_msg.position[2] = -1.57;
                mani_ctrl_pub.publish(mani_ctrl_msg);

                nStep = STEP_DONE;
                ROS_WARN("[STEP_DONE] x= %.2f y= %.2f " ,pose_diff.x, pose_diff.y);
            }

            result_msg.data = "backward";
            result_pub.publish(result_msg);
        }

         //7、抓取任务完毕
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

    delete tf_listener; 

    return 0;

}