#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "WPR1_driver.h"

static CWPR1_driver m_wpr1;
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("[cmdVelCallback] liner(%.2f %.2f) angular(%.2f)", msg->linear.x,msg->linear.y,msg->angular.z);
    m_wpr1.Velocity(msg->linear.x,msg->linear.y,msg->angular.z);
}

static int nFirstVal[4];
static bool bFirst = true;
int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpr1_motor_encoder");
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel",1,&cmdVelCallback);

    ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("serial_port", strSerialPort, "/dev/ttyUSB0");
    m_wpr1.Open(strSerialPort.c_str(),115200);
    ROS_WARN("[wpr1_motor_encoder] dev = %s ",strSerialPort.c_str());
    
    ros::Rate r(100.0);
    r.sleep();

    while(n.ok())
    {
        m_wpr1.ReadNewData();

        ///////////////////
        //ROS_INFO("enc [M1]%d [M2]%d [M3]%d ", m_wpr1.arMotorPos[0], m_wpr1.arMotorPos[1], m_wpr1.arMotorPos[2]);
        ////////////////////
        if(bFirst == true)
        {
            nFirstVal[0] = m_wpr1.arMotorPos[0];
            nFirstVal[1] = m_wpr1.arMotorPos[1];
            nFirstVal[2] = m_wpr1.arMotorPos[2];
            bFirst = false;
        }
        else
        {
            int nDiff[4];
            nDiff[0] = m_wpr1.arMotorPos[0] - nFirstVal[0];
            nDiff[1] = m_wpr1.arMotorPos[1] - nFirstVal[1];
            nDiff[2] = m_wpr1.arMotorPos[2] - nFirstVal[2];
            ROS_INFO("Encoder [M1]%d [M2]%d [M3]%d", m_wpr1.arMotorPos[0], m_wpr1.arMotorPos[1], m_wpr1.arMotorPos[2]);
            //ROS_INFO("Diff [M1]%d [M2]%d [M3]%d", nDiff[0], nDiff[1], nDiff[2]);
        }
        /////////////////////
        
        ros::spinOnce();
        r.sleep();
    }
}