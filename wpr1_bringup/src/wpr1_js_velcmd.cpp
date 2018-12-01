#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>


using namespace std;

class TeleopJoy
{
public:
  TeleopJoy();
  float lx;
  float ry;
  float ly;
  ros::NodeHandle n;
  ros::Subscriber sub;

  ros::Time current_time;
  ros::Time last_time;
  ros::Publisher velcmd_pub;
private:
  void callBack(const sensor_msgs::Joy::ConstPtr& joy);
};

TeleopJoy::TeleopJoy()
{
  lx = 0;
  ry = 0;
  velcmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
  sub = n.subscribe<sensor_msgs::Joy>("joy",10,&TeleopJoy::callBack,this);
  current_time = ros::Time::now();
  last_time = ros::Time::now();
 
  ROS_INFO("TeleopJoy");
}

static float kx = 0.2;
static float ky = 0.1;
static float kz = 0.5;
void TeleopJoy::callBack(const sensor_msgs::Joy::ConstPtr& joy)
{

  ROS_INFO("Joy: [%.2f , %.2f]", lx , ry);
  lx = joy->axes[1];
  ly = joy->axes[0];
  ry = joy->axes[3];

  geometry_msgs::Twist vel_cmd;
  vel_cmd.linear.x = (float)lx*kx;
  vel_cmd.linear.y = ly*ky;
  vel_cmd.linear.z = 0;
  vel_cmd.angular.x = 0;
  vel_cmd.angular.y = 0;
  vel_cmd.angular.z = (float)ry*kz;
  velcmd_pub.publish(vel_cmd);
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wpr1_js_velcmd");

  TeleopJoy cTeleopJoy;

  ros::spin();

  return 0;
}
