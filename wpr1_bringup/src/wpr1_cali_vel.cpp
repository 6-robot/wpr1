#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class CVelocityTest
{
public:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;

  //! ROS node initialization
  CVelocityTest(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  }

  //! Loop forever while sending drive commands based on keyboard input
  bool driveKeyboard()
  {
    std::cout << "Type a command and then press enter.  "
      "Use 'w' to move forward, 's' to move backward, "
      "Use 'a' to shift left, 'd' to shift right, "
      "'q' to turn left 'e' to turn right, 'x' to exit.\n";

    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;

    char cmd[50];
    while(nh_.ok())
    {
      std::cin.getline(cmd, 50);
      if(cmd[0]!='w' && cmd[0]!='a' && cmd[0]!='s' && cmd[0]!='d' && cmd[0]!='q' && cmd[0]!='e' && cmd[0]!='x')
      {
        std::cout << "unknown command:" << cmd << "\n";
        continue;
      }

      int nDelay = 0;
      base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
      //move forward
      if(cmd[0]=='w')
      {
        base_cmd.linear.x = 0.25;
        nDelay = 4;
      } 
      if(cmd[0]=='s')
      {
        base_cmd.linear.x = -0.25;
        nDelay = 4;
      } 
      //shift left 
      else if(cmd[0]=='a')
      {
        base_cmd.linear.y = 0.25;
        nDelay = 4;
      } 
      //shift right 
      else if(cmd[0]=='d')
      {
        base_cmd.linear.y = -0.25;
        nDelay = 4;
      } 
      //turn left (yaw) 
      else if(cmd[0]=='q')
      {
        base_cmd.angular.z = 3.1415926/4; 
        nDelay = 8;
      } 
      //turn right (yaw) 
      else if(cmd[0]=='e')
      {
        base_cmd.angular.z = -3.1415926/4;
        nDelay = 8;
      } 
      //quit
      else if(cmd[0]=='x')
      {
        break;
      }

      while(nDelay > 0)
      {
          std::cout << "Count = " << nDelay << "\n";
          nDelay --;
          //publish the assembled command
          cmd_vel_pub_.publish(base_cmd);
          sleep(1);     
      }
      
      base_cmd.linear.x = 0;
      base_cmd.linear.y = 0;
      base_cmd.angular.z = 0;
      cmd_vel_pub_.publish(base_cmd);
      
      std::cout << "Action Done!\n\nNext CMD:";
    }
    return true;
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "wpr1_calibrate_velocity");
  ros::NodeHandle nh;

  CVelocityTest vt(nh);
  vt.driveKeyboard();

}