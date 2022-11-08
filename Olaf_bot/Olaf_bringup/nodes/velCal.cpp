#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Olaf_bringup/To_odom.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

double vel_y = 0;
double vel_th = 0;
double x = 0.0;
double y = 0.0;
double th = 0.0;
double vel_r = 0;
double vel_l = 0;
double L = 0.4104;


ros::Time current_time, last_time;

void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    vel_y = msg->linear.y;
    vel_th = msg->angular.z;
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    th = msg->pose.pose.orientation.z;
}

void Cal_Vel(float vel_y, float vel_th){
  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec(); //sub주기로 타임스텝 맞춰야함
  last_time = current_time;
  double del_y = vel_y*dt;
  double del_th = vel_th*dt;

  double ds_r = ((2*del_y)/sin(th + del_th/2) + L*del_th)/2;
  double ds_l = ((2*del_y)/sin(th + del_th/2) - L*del_th)/2;

  vel_r = ds_r/dt;
  vel_l = ds_l/dt;

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher odom_node = n.advertise<Olaf_bringup::To_odom>("wheel_vel", 10);
  ros::Subscriber sub_cmdVel = n.subscribe("cmd_vel", 10, CmdVelCallback);
  ros::Subscriber sub_odom = n.subscribe("odom", 10, OdomCallback);

  ros::Rate loop_rate(10);
  Olaf_bringup::To_odom to_odom;
  nav_msgs::Odometry odom;
  geometry_msgs::Twist cmd;
  
  int count = 0;
  while (ros::ok())
  {
 
    Cal_Vel(vel_y, vel_th);
    to_odom.velL = vel_r;
    to_odom.velR = vel_l;

    odom_node.publish(to_odom);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
