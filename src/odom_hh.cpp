#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

double x = 0.0;
double y = 0.0;
double th = 0.0;
  
double ds = 0;  //로봇의 주행 거리
double L = 0.4104;  //차체의 길이

double delta_th = 0;
double delta_x = 0;
double delta_y = 0;

double ds_r = 0;
double ds_l = 0;

double vx = 0;
double vy = 0;
double vth = 0;

double dv_r = 0;
double dv_l = 0;

ros::Time current_time, last_time;

void Update_Odom(double dv_r, double dv_l){
  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec(); //sub주기로 타임스텝 맞춰야함
  ds_r = dv_r*dt;
  ds_l = dv_l*dt;

  //얼마나 움직였는 지에 대한 설정
  ds = (ds_r+ds_l)/2;
  delta_th = (ds_r-ds_l)/L;

  delta_x = ds*cos(th+delta_th/2);
  delta_y = ds*sin(th+delta_th/2);

  vx = delta_x/dt;
  vy = delta_y/dt;
  vth = delta_th/dt;
  last_time = ros::Time::now();
}

void wheelVelCallback(const std_msgs::String::ConstPtr& msg){ //승인이의 바퀴 정보 받아오기
 // msg->data. = dv_r, dv_l;
  Update_Odom(dv_r, dv_l);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber sub = n.subscribe("wheel_vel", 1000, wheelVelCallback);
  tf::TransformBroadcaster odom_broadcaster;
 

  ros::Time current_time, last_time;


  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
 

    //compute odometry in a typical way given the velocities of the robot
    x += delta_x;
    y += delta_y;
    th += delta_th;


    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}