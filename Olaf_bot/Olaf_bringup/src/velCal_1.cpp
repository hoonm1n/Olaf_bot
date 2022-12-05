#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Olaf_bringup/To_odom.h>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

double vel_x = 0;
double vel_th = 0;
double x = 0.0;
double y = 0.0;
double th = 0.1;
double vel_r = 0;
double vel_l = 0;
double L = 0.4104;
double pi = M_PI;

ros::Time current_time, last_time;

void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    vel_x = msg->linear.x;
    vel_th = msg->angular.z;
    
}

void OdomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    th = msg->pose.pose.orientation.z;
}

void Cal_Vel(float vel_x, float vel_th){
  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec(); //sub주기로 타임스텝 맞춰야함
  last_time = current_time;
  double del_x = vel_x*dt;
  double del_th = vel_th*dt;

  double ds_r = ((2*del_x)/cos(th + del_th/2) + L*del_th)/2;
  double ds_l = ((2*del_x)/cos(th + del_th/2) - L*del_th)/2;

  vel_r = ds_r/dt;
  vel_l = ds_l/dt;

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher odom_node = n.advertise<Olaf_bringup::To_odom>("goalvel", 10);
  ros::Subscriber sub_cmdVel = n.subscribe("cmd_vel", 10, CmdVelCallback);
  ros::Subscriber sub_odom = n.subscribe("amcl_pose", 10, OdomCallback);

  ros::Rate loop_rate(10);
  Olaf_bringup::To_odom to_odom;
  geometry_msgs::PoseWithCovarianceStamped odom;
  geometry_msgs::Twist cmd;
  
  int count = 0;
  while (ros::ok())
  {
 
    Cal_Vel(vel_x, vel_th);
    to_odom.velL = vel_l;
    to_odom.velR = vel_r;

    odom_node.publish(to_odom);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}