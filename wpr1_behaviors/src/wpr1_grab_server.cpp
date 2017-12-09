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
#include "highgui.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>
#include "obj_track.h"

#define STEP_WAIT           0
#define STEP_FIND_PLANE     1
#define STEP_PLANE_HEIGHT   2
#define STEP_PLANE_DIST     3
#define STEP_FIND_OBJ       4
#define STEP_OBJ_Y          5
#define STEP_OBJ_X          6
#define STEP_HAND_UP        7
#define STEP_FORWARD        8
#define STEP_GRAB           9
#define STEP_OBJ_UP         10
#define STEP_BACKWARD       11
#define STEP_DONE           12
static int nStep = STEP_WAIT;

static std::string pc_topic;
static ros::Publisher pc_pub;
static ros::Publisher marker_pub;
static ros::Publisher vel_pub;
static ros::Publisher joint_ctrl_pub;
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

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
ros::Publisher segmented_plane;
ros::Publisher segmented_objects;
ros::Publisher masking;
ros::Publisher color;
static sensor_msgs::JointState ctrl_msg;
static std_msgs::String result_msg;

static CObjTrack obj_track;
cv::Mat rgb_image;

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

static int nTimeDelayCounter = 0;
static int nFrameCount = 0;
static float fLastPlaneHeight = 0.5;
static float fPlaneHeight = 0;
static int nPlaneHeightCounter = 0;

static float fPlaneDist = 0;
static float fTargetPlaneDist = 0.6;    //与桌子之间的目标距离
static float fTargetGrabX = 0.8;        //抓取时目标物品的x坐标
static float fTargetGrabY = 0.05;          //抓取时目标物品的y坐标

static std::vector<stBoxMarker> vObj;        
static stBoxMarker boxLastObject;
static int nObjDetectCounter = 0;

void ProcCloudCB(const sensor_msgs::PointCloud2 &input)
{
    //ROS_WARN("ProcCloudCB");

    //to footprint
    sensor_msgs::PointCloud2 pc_footprint;
    tf_listener->waitForTransform("/base_footprint", input.header.frame_id, input.header.stamp, ros::Duration(5.0));  //return value always  false!
    pcl_ros::transformPointCloud("/base_footprint", input, pc_footprint, *tf_listener);

    //source cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
    pcl::fromROSMsg(pc_footprint , cloud_src);
    //ROS_INFO("cloud_src size = %d",cloud_src.size()); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_ptr;
    //////////////////////////////////////////////////
    //1、原始点云
    //cloud_source_ptr = cloud_src.makeShared(); 
    /////////////////////////////////////////////////
    //2、新构建一个点云，只取一部分
    pcl::PointCloud<pcl::PointXYZRGB> new_point_cloud;
    int nSize = cloud_src.points.size();
    for(int i=0;i<nSize;i++)
    {
        pcl::PointXYZRGB p = cloud_src.points[i];
        if(
            p.x < 1.2 &&
            p.y > -1.0 &&
            p.y < 1.0
        )
        {
            new_point_cloud.push_back(p);
        }
    }
    cloud_source_ptr = new_point_cloud.makeShared(); 
    //////////////////////////////////////////////////
    
    /*一、数据处理 *************************************************************************************************/
    if( nStep == STEP_FIND_PLANE /*|| nStep == STEP_OBJ_X || nStep == STEP_OBJ_Y|| nStep == STEP_PLANE_HEIGHT || nStep == STEP_PLANE_DIST*/ || nStep == STEP_FIND_OBJ)
    {
        // Process
        //ROS_INFO("Cloud: width = %d, height = %d\n", input.width, input.height);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Get the plane model, if present.
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
        segmentation.setInputCloud(cloud_source_ptr);
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setDistanceThreshold(0.005);
        segmentation.setOptimizeCoefficients(true);
        pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
        segmentation.segment(*planeIndices, *coefficients);
        //ROS_INFO_STREAM("Num_Of_Planes = " << planeIndices->indices.size());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;

        RemoveBoxes();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
        int i = 0, nr_points = (int) cloud_source_ptr->points.size ();
        // While 30% of the original cloud is still there
        while (cloud_source_ptr->points.size () > 0.03 * nr_points)
        {
            // Segment the largest planar component from the remaining cloud
            segmentation.setInputCloud (cloud_source_ptr);
            segmentation.segment (*planeIndices, *coefficients);
            if (planeIndices->indices.size () == 0)
            {
                ROS_WARN("Could not estimate a planar model for the given dataset.");
                break;
            }

            // Extract the planeIndices
            extract.setInputCloud (cloud_source_ptr);
            extract.setIndices (planeIndices);
            extract.setNegative (false);
            extract.filter (*plane);
            float plane_height = 0;
            for(int k=0; k<plane->width * plane->height ;k ++ )
            {
                plane_height += plane->points[k].z;
            }
            plane_height /= plane->width * plane->height;
            //ROS_WARN("%d - plane: %d points. height =%.2f" ,i, plane->width * plane->height,plane_height);
            fPlaneHeight = plane_height;
           
            bool bFirstPoint = true;
            for (int j = 0; j < plane->points.size(); j++) 
            {
                pcl::PointXYZRGB p = plane->points[j];
                if(bFirstPoint == true)
                {
                    boxPlane.xMax = boxPlane.xMin = p.x;
                    boxPlane.yMax = boxPlane.yMin = p.y;
                    boxPlane.zMax = boxPlane.zMin = p.z;
                    bFirstPoint = false;
                }

                if(p.x < boxPlane.xMin) { boxPlane.xMin = p.x;}
                if(p.x > boxPlane.xMax) { boxPlane.xMax = p.x;}
                if(p.y < boxPlane.yMin) { boxPlane.yMin = p.y;}
                if(p.y > boxPlane.yMax) { boxPlane.yMax = p.y;}
                if(p.z < boxPlane.zMin) { boxPlane.zMin = p.z;}
                if(p.z > boxPlane.zMax) { boxPlane.zMax = p.z;}
            } 
            //////////////////////////////////
            //测试：将平面用框框框起来
            DrawBox(boxPlane.xMin, boxPlane.xMax, boxPlane.yMin, boxPlane.yMax, boxPlane.zMin, boxPlane.zMax, 1, 0, 1);
            //ROS_WARN("[FIND_PLANE] x= (%.2f , %.2f) y=(%.2f , %.2f) z=(%.2f , %.2f)" ,boxPlane.xMin,boxPlane.xMax,boxPlane.yMin,boxPlane.yMax,boxPlane.zMin,boxPlane.zMax);
            ///////////////////////////////////
            if(plane_height > 0.6 && plane_height < 0.85) 
            break;

            // Create the filtering object
            extract.setNegative (true);
            extract.filter (*cloud_f);
            cloud_source_ptr.swap (cloud_f);
            i++;
        }

        if (planeIndices->indices.size() == 0)
            std::cout << "Could not find a plane in the scene." << std::endl;
        else
        {
            // Copy the points of the plane to a new cloud.
            extract.setInputCloud(cloud_source_ptr);
            extract.setIndices(planeIndices);
            extract.filter(*plane);

            // Retrieve the convex hull.(生成凸包)
            pcl::ConvexHull<pcl::PointXYZRGB> hull;
            hull.setInputCloud(plane);
            // Make sure that the resulting hull is bidimensional.
            hull.setDimension(2);
            hull.reconstruct(*convexHull);

            // Redundant check.
            if (hull.getDimension() == 2)
            {
                segmented_plane.publish(plane); //在RVIZ里显示平面
                if(nStep == STEP_FIND_OBJ /*|| nStep == STEP_OBJ_X || nStep == STEP_OBJ_Y8*/)
                {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);
                    // Prism object.(用于分割出棱柱模型内部的点集)
                    pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
                    prism.setInputCloud(cloud_source_ptr);
                    prism.setInputPlanarHull(convexHull);
                    prism.setHeightLimits(-0.30, -0.05); //height limit objects lying on the plane (物体高度范围)
                    pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

                    //Get and show all points retrieved by the hull.
                    prism.segment(*objectIndices);
                    extract.setIndices(objectIndices);
                    extract.filter(*objects);
                    segmented_objects.publish(objects); //所有物品的点集

                    // run clustering extraction on "objects" to get several pointclouds
                    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
                    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
                    std::vector<pcl::PointIndices> cluster_indices;
                    ec.setClusterTolerance (0.02);  //近邻搜索的搜索半径
                    ec.setMinClusterSize (500);     //单个聚类需要的最少点数目
                    ec.setMaxClusterSize (10000000);//单个聚类需要的最多点数目
                    ec.setSearchMethod (tree);      //设置点云的搜索机制
                    ec.setInputCloud (objects);     //输入物品集点云
                    ec.extract (cluster_indices);   //从点云中提取聚类

                    pcl::ExtractIndices<pcl::PointXYZRGB> extract_object_indices;//创建物品提取对象
                    std::vector<pcl::PointCloud<pcl::PointXYZRGB> > objectf;
                    cv::Mat element;

                    vObj.clear();
                    int nObjCnt = 0;
                    for(int i = 0; i<cluster_indices.size(); ++i)
                    {
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                        extract_object_indices.setInputCloud(objects);          //将物品集点云全部输入
                        extract_object_indices.setIndices(boost::make_shared<const pcl::PointIndices>(cluster_indices[i]));
                        extract_object_indices.filter(*object_cloud);           //提取输出存储到object_cloud
                        objectf.push_back(*object_cloud);

                        //统计该物品点云的空间范围
                        bool bFirstPoint = true;
                        for (int j = 0; j < object_cloud->points.size(); j++) 
                        {
                            pcl::PointXYZRGB p = object_cloud->points[j];
                            if(bFirstPoint == true)
                            {
                                boxMarker.xMax = boxMarker.xMin = p.x;
                                boxMarker.yMax = boxMarker.yMin = p.y;
                                boxMarker.zMax = boxMarker.zMin = p.z;
                                bFirstPoint = false;
                            }

                            if(p.x < boxMarker.xMin) { boxMarker.xMin = p.x;}
                            if(p.x > boxMarker.xMax) { boxMarker.xMax = p.x;}
                            if(p.y < boxMarker.yMin) { boxMarker.yMin = p.y;}
                            if(p.y > boxMarker.yMax) { boxMarker.yMax = p.y;}
                            if(p.z < boxMarker.zMin) { boxMarker.zMin = p.z;}
                            if(p.z > boxMarker.zMax) { boxMarker.zMax = p.z;}

                        }
                        if(boxMarker.xMin < 1.5 && boxMarker.yMin > -0.5 && boxMarker.yMax < 0.5)   //物品所处的空间限定
                        {
                            DrawBox(boxMarker.xMin, boxMarker.xMax, boxMarker.yMin, boxMarker.yMax, boxMarker.zMin, boxMarker.zMax, 0, 1, 0);

                            std::ostringstream stringStream;
                            stringStream << "obj_" << nObjCnt;
                            std::string obj_id = stringStream.str();
                            DrawText(obj_id,0.08, boxMarker.xMax,(boxMarker.yMin+boxMarker.yMax)/2,boxMarker.zMax + 0.04, 1,0,1);
                            nObjCnt++;
                            //ROS_WARN("[obj_%d] xMin= %.2f yMin = %.2f yMax = %.2f",i,boxMarker.xMin, boxMarker.yMin, boxMarker.yMax);
                            vObj.push_back(boxMarker);
                        } 
                    }
                    //对物品检测结果进行处理
                    if(nObjCnt > 0)
                    {
                        if(nObjDetectCounter == 0)
                        {
                            //第一帧记录最靠中间的物品
                            int nNumObj = vObj.size();
                            if(nNumObj > 0)
                            {
                                float fMidY = (vObj[0].yMin + vObj[0].yMax)/2;
                                boxLastObject = vObj[0];
                                for(int i=0;i<nNumObj;i++)
                                {
                                    float fTmpMidY = (vObj[i].yMin + vObj[i].yMax)/2;
                                    if(fabs(fTmpMidY) < fabs(fMidY))
                                    {
                                        boxLastObject = vObj[i];
                                    }
                                }
                                nObjDetectCounter ++;
                            }
                            else
                            {
                                ROS_WARN("[4_FIND_OBJ] nNumObj <= 0 ...");  //不会有效果
                            }
                        }
                        else
                        {
                            //后面的帧检测之前记录的物品还在不在
                            float fMidY = (boxLastObject.yMin + boxLastObject.yMax)/2;
                            int nNearIndex = 0;
                            float fNearDiff = fabs((vObj[0].yMin + vObj[0].yMax)/2 - fMidY);
                            int nNumObj = vObj.size();
                            if(nNumObj > 0)
                            {
                                for(int i=0;i<nNumObj;i++)
                                {
                                    float fTmpMidY = (vObj[i].yMin + vObj[i].yMax)/2;
                                    float fTmpDiff = fabs(fTmpMidY - fMidY);
                                    //ROS_WARN("[4_FIND_OBJ] fTmpMidY = %.2f  fTmpDiff = %.2f",fTmpMidY,fTmpDiff);
                                    if( fabs(fTmpDiff) <= fNearDiff )
                                    {
                                        boxLastObject = vObj[i];
                                        fNearDiff = fabs(fTmpDiff);
                                        if(nStep == STEP_FIND_OBJ)
                                        {
                                            nObjDetectCounter ++;
                                            ROS_WARN("[4_FIND_OBJ] nObjDetectCounter = %d",nObjDetectCounter);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        nObjDetectCounter = 0;
                        ROS_WARN("[4_FIND_OBJ] nObjCnt <= 0 No object detected ...");
                    }
                }
                else
                {
                    //不需要检测物品时，仅仅发布平面数据即可
                    segmented_plane.publish(plane); 
                }
                

            }
            else std::cout << "The chosen hull is not planar." << std::endl;
        }
    }

    /*二、有限状态机 *************************************************************************************************/
    //1、统计识别次数，确认平面
    if(nStep == STEP_FIND_PLANE)  //(点云处理算法在上面)
    {
        VelCmd(0,0,0);
        if(fabs(fPlaneHeight - fLastPlaneHeight) < 0.05)
        {
            nPlaneHeightCounter ++;
        }
        else
        {
            nPlaneHeightCounter = 0;
        }
        fLastPlaneHeight = fPlaneHeight;
        ROS_WARN("[1_FIND_PLANE] z= %.2f xm= %.2f y=(%.2f , %.2f) counter= %d" ,fPlaneHeight,boxPlane.xMin,boxPlane.yMin,boxPlane.yMax,nPlaneHeightCounter);
        if(nPlaneHeightCounter > 10)
        {
            nPlaneHeightCounter = 0;
            nTimeDelayCounter = 0;
            nStep = STEP_PLANE_HEIGHT;
        }
        result_msg.data = "find plane";
        result_pub.publish(result_msg);
    }
    //2、根据平面高度调节脊柱升降高度
    if(nStep == STEP_PLANE_HEIGHT)
    {
        if(nTimeDelayCounter == 0)
        {
            float fTorsoVal = fPlaneHeight + 0.64 -1.3;
            if (fTorsoVal < 0)
			{
				fTorsoVal = 0;
			}
			if (fTorsoVal > 0.35)
			{
				fTorsoVal = 0.35;
			}
            ctrl_msg.position[0] = fTorsoVal;
            joint_ctrl_pub.publish(ctrl_msg);
            ROS_WARN("[2_PLANE_HEIGHT] fTorsoVal = %.2f " ,fTorsoVal);
            result_msg.data = "plane height";
            result_pub.publish(result_msg);
        }

        nTimeDelayCounter ++;
        if(nTimeDelayCounter > 90)
        {
            nStep = STEP_PLANE_DIST;
        }

    }
    //3、前后运动控制到平面的距离
    if(nStep == STEP_PLANE_DIST)
    {
        float fMinDist = 100;
        for (int i = 0; i < cloud_source_ptr->points.size(); i++) 
        {
            pcl::PointXYZRGB p = cloud_source_ptr->points[i];
            if(
                p.y > -0.2 && p.y < 0.2 && 
                p.z > fPlaneHeight-0.05 && p.z < fPlaneHeight+0.05 && 
                p.x < fMinDist)
            {
                fMinDist = p.x;
            }
        }
        fPlaneDist = fMinDist;
        ROS_WARN("[3_PLANE_DIST] dist= %.2f" ,fPlaneDist);
        float diff = fPlaneDist - fTargetPlaneDist;
        if(fabs(diff) < 0.02)
        {
            nObjDetectCounter = 0;
            nStep = STEP_FIND_OBJ;
            VelCmd(0,0,0);
        } 
        else
        {
            if(diff > 0)
            {
                //距离还太远，前进
                VelCmd(0.1,0,0);
            }
            else
            {
                //距离还太近，后退
                VelCmd(-0.1,0,0);
            }
        }

        result_msg.data = "plane dist";
        result_pub.publish(result_msg);
    }
    //4、检测物品，挑选出准备抓取的目标物品
    if(nStep == STEP_FIND_OBJ)  //(点云处理算法在上面)
    {
        VelCmd(0,0,0);
        if(nObjDetectCounter > 3)
        {
            nObjDetectCounter = 0;
            //把目标物品的坐标赋值到锁定对象里
            obj_track.obj_x = boxLastObject.xMin;
            obj_track.obj_y = (boxLastObject.yMin+boxLastObject.yMax)/2;
            obj_track.obj_z = boxLastObject.zMax;
            nStep = STEP_OBJ_X;
        }

        result_msg.data = "find objects";
        result_pub.publish(result_msg);
    }

    //5、根据目标物品坐标调节前后距离 
    if(nStep == STEP_OBJ_X)
    {
        int nSize = cloud_source_ptr->points.size();
        for (int i = 0; i < nSize; i++)
        {
            obj_track.pntx[i] = cloud_source_ptr->points[i].x;
            obj_track.pnty[i] = cloud_source_ptr->points[i].y;
            obj_track.pntz[i] = cloud_source_ptr->points[i].z;
        }
        obj_track.nPointNum = nSize;
        obj_track.GetObjPosition();
        float x_diff = obj_track.obj_x - fTargetGrabX;
        if(fabs(x_diff) < 0.02)
        {
            nStep = STEP_OBJ_Y;
            VelCmd(0,0,0);
        }
        else
        {
            if(x_diff > 0)
            {
                //目标还太远，靠近一些
                VelCmd(0.05,0,0);
            }
            else
            {
                //目标还太近，远离一些
                VelCmd(-0.05,0,0);
            }
        }
        ROS_WARN("[5_STEP_OBJ_X] p_h = %.2f ox= %.2f ,tx= %.2f xd= %.2f" ,obj_track.fPlaneHeight, obj_track.obj_x, fTargetGrabX,x_diff);
        RemoveBoxes();
        DrawBox(obj_track.obj_x, obj_track.obj_x + 0.1, obj_track.obj_y-0.05, obj_track.obj_y+0.05, obj_track.fPlaneHeight, obj_track.obj_z, 1, 1, 0);

        result_msg.data = "object x";
        result_pub.publish(result_msg);
    }
    //6、根据目标物品坐标调节左右距离
    if(nStep == STEP_OBJ_Y)
    {
       int nSize = cloud_source_ptr->points.size();
        for (int i = 0; i < nSize; i++)
        {
            obj_track.pntx[i] = cloud_source_ptr->points[i].x;
            obj_track.pnty[i] = cloud_source_ptr->points[i].y;
            obj_track.pntz[i] = cloud_source_ptr->points[i].z;
        }
        obj_track.nPointNum = nSize;
        obj_track.GetObjPosition();
        //obj_track.obj_y = (boxLastObject.yMin+boxLastObject.yMax)/2;
        float y_diff = obj_track.obj_y - fTargetGrabY;
        if(fabs(y_diff) < 0.01)
        {
            nTimeDelayCounter = 0;
            nStep = STEP_HAND_UP;
            VelCmd(0,0,0);
        }
        else
        {
            if(y_diff > 0)
            {
                //目标靠左，左移一些
                VelCmd(0,0.05,0);
            }
            else
            {
                //目标靠右，右移一些
                VelCmd(0,-0.05,0);
            }
        }
        ROS_WARN("[6_STEP_OBJ_Y] p_h = %.2f oy= %.2f ,ty= %.2f (yd= %.2f)" ,obj_track.fPlaneHeight, obj_track.obj_y, fTargetGrabY,y_diff);
        RemoveBoxes();
        DrawBox(obj_track.obj_x, obj_track.obj_x + 0.1, obj_track.obj_y-0.05, obj_track.obj_y+0.05, obj_track.fPlaneHeight, obj_track.obj_z, 1, 1, 0);

        result_msg.data = "object y";
        result_pub.publish(result_msg);
    }
    //7、抬起手臂
    if(nStep == STEP_HAND_UP)
    {
        if(nTimeDelayCounter == 0)
        {
            ctrl_msg.position[1] = -0.52;
            ctrl_msg.position[2] = -0.52;
            ctrl_msg.position[3] = 0;
            ctrl_msg.position[4] = 47000;
            joint_ctrl_pub.publish(ctrl_msg);
            ROS_WARN("[7_HAND_UP] gripper = %.2f " ,ctrl_msg.position[4]);

            result_msg.data = "hand up";
            result_pub.publish(result_msg);
        }
        nTimeDelayCounter ++;
        if(nTimeDelayCounter > 150)
        {
            nTimeDelayCounter = 0;
            nStep = STEP_FORWARD;
        }
    }
    //8、前进靠近物品
    if(nStep == STEP_FORWARD)
    {
        ROS_WARN("[8_STEP_FORWARD] nTimeDelayCounter = %d " ,nTimeDelayCounter);
        nTimeDelayCounter++;
        VelCmd(0.1,0,0);
        if(nTimeDelayCounter > 60)
        {
            VelCmd(0,0,0);
            
            nTimeDelayCounter = 0;
            nStep = STEP_GRAB;
        }

        result_msg.data = "forward";
        result_pub.publish(result_msg);
    }
    //9、抓取物品
    if(nStep == STEP_GRAB)
    {
        if(nTimeDelayCounter == 0)
        {
            ctrl_msg.position[4] = 7000;
            joint_ctrl_pub.publish(ctrl_msg);
            ROS_WARN("[9_STEP_GRAB] gripper = %.2f " ,ctrl_msg.position[4]);

            result_msg.data = "grab";
            result_pub.publish(result_msg);
        }
        nTimeDelayCounter++;
        VelCmd(0,0,0);
        if(nTimeDelayCounter > 120)
        {
            nTimeDelayCounter = 0;
            nStep = STEP_OBJ_UP;
        }
    }
    //10、拿起物品
    if(nStep == STEP_OBJ_UP)
    {
        if(nTimeDelayCounter == 0)
        {
            ctrl_msg.position[0] += 0.05;
            joint_ctrl_pub.publish(ctrl_msg);
            ROS_WARN("[10_STEP_OBJ_UP] fTorsoVal = %.2f " ,ctrl_msg.position[0]);

            result_msg.data = "object up";
            result_pub.publish(result_msg);
        }
        nTimeDelayCounter++;
        VelCmd(0,0,0);
        if(nTimeDelayCounter > 60)
        {
            nTimeDelayCounter = 0;
            nStep = STEP_BACKWARD;
        }
    }
    //11、带着物品后退
    if(nStep == STEP_BACKWARD)
    {
        ROS_WARN("[11_STEP_BACKWARD] nTimeDelayCounter = %d " ,nTimeDelayCounter);
        nTimeDelayCounter++;
        VelCmd(-0.1,0,0);
        if(nTimeDelayCounter > 70)
        {
            nTimeDelayCounter = 0;
            VelCmd(0,0,0);
            ctrl_msg.position[1] = -1.57;
            ctrl_msg.position[2] = -1.57;
            joint_ctrl_pub.publish(ctrl_msg);
            nStep = STEP_DONE;
        }

        result_msg.data = "backward";
        result_pub.publish(result_msg);
    }
    //12、抓取任务完毕
    if(nStep == STEP_DONE)
    {
        result_msg.data = "done";
        result_pub.publish(result_msg);
    }
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
    ctrl_msg.position[0] = inTorso;
    ctrl_msg.position[1] = inShoulder;
    ctrl_msg.position[2] = inElbow;
    ctrl_msg.position[3] = inWrist;
    ctrl_msg.position[4] = inGripper;
    joint_ctrl_pub.publish(ctrl_msg);
}

void BehaviorCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("grab start");
    if( nFindIndex >= 0 )
    {
       ctrl_msg.position[1] = -1.57;
        ctrl_msg.position[2] = -0.7;
        ctrl_msg.position[3] = 0;
        joint_ctrl_pub.publish(ctrl_msg);
        VelCmd(0,0,0);
        nStep = STEP_FIND_PLANE;
        ROS_WARN("[grab_start] ");
    }

    nFindIndex = msg->data.find("grab stop");
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
    ros::init(argc, argv, "wpr1_grab_server");
    ROS_INFO("wpr1_grab_server");
    tf_listener = new tf::TransformListener(); 

    ros::NodeHandle nh_param("~");
    nh_param.param<std::string>("topic", pc_topic, "/kinect2/sd/points");
    bool bActive = false;
    nh_param.param<bool>("start", bActive, false);
    if(bActive == true)
    {
        ctrl_msg.position[1] = -1.57;
        ctrl_msg.position[2] = -0.7;
        ctrl_msg.position[3] = 0;
        joint_ctrl_pub.publish(ctrl_msg);
        VelCmd(0,0,0);
        nStep = STEP_FIND_PLANE;
    }

    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe(pc_topic, 10 , ProcCloudCB);

    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("obj_pointcloud",1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("obj_marker", 10);

    segmented_objects = nh.advertise<PointCloud> ("segmented_objects",1);
    segmented_plane = nh.advertise<PointCloud> ("segmented_plane",1);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    joint_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpr1/joint_ctrl", 30);
    result_pub = nh.advertise<std_msgs::String>("/wpr1/grab_result", 30);

    ros::Subscriber sub_sr = nh.subscribe("/wpr1/behaviors", 30, BehaviorCB);

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
    ctrl_msg.position[4] = 7000;
    ctrl_msg.velocity[0] = 1500;
    ctrl_msg.velocity[1] = 1500;
    ctrl_msg.velocity[2] = 1500;
    ctrl_msg.velocity[3] = 1500;
    ctrl_msg.velocity[4] = 1500;

    ros::Rate r(30);
    while(nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    delete tf_listener; 

    return 0;

}