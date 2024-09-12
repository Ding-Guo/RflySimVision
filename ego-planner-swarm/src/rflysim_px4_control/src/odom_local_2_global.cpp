#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "quadrotor_msgs/PositionCommand.h"

ros::Subscriber _odom_sub;
ros::Publisher  _odom_pub;

//quadrotor_msgs::PositionCommand _cmd;
nav_msgs::Odometry _odom;
double _init_x, _init_y, _init_z;

bool rcv_odom = false;
void rcvOdomCallBack(const nav_msgs::Odometry odom)
{
    rcv_odom = true;
    _odom    = odom;
}

void pubOdom()
{
    nav_msgs::Odometry odom;
    odom.header.stamp    = ros::Time::now();
    odom.header.frame_id = "world";

    if(rcv_odom)
    {
        odom.header.stamp = _odom.header.stamp;
        odom.pose.pose.position.x = _odom.pose.pose.position.x + _init_x;
        odom.pose.pose.position.y = _odom.pose.pose.position.y + _init_y;
        odom.pose.pose.position.z = _odom.pose.pose.position.z + _init_z;

        odom.pose.pose.orientation.w = _odom.pose.pose.orientation.w;
        odom.pose.pose.orientation.x = _odom.pose.pose.orientation.x;
        odom.pose.pose.orientation.x = _odom.pose.pose.orientation.x;
        odom.pose.pose.orientation.z = _odom.pose.pose.orientation.z;

        odom.twist.twist.linear.x = _odom.twist.twist.linear.x;
        odom.twist.twist.linear.y = _odom.twist.twist.linear.y;
        odom.twist.twist.linear.z = _odom.twist.twist.linear.z;

        odom.twist.twist.angular.x = _odom.twist.twist.angular.x;
        odom.twist.twist.angular.y = _odom.twist.twist.angular.y;
        odom.twist.twist.angular.z = _odom.twist.twist.angular.z;
    }
    else
    {
        odom.pose.pose.position.x = _init_x;
        odom.pose.pose.position.y = _init_y;
        odom.pose.pose.position.z = _init_z;

        odom.pose.pose.orientation.w = -0.7;
        odom.pose.pose.orientation.x = 0;
        odom.pose.pose.orientation.y = 0;
        odom.pose.pose.orientation.z = -0.7;

        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;

        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = 0.0;
    }

    _odom_pub.publish(odom);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "odom_generator");
    ros::NodeHandle nh( "~" );

    nh.param("init_x", _init_x,  0.0);
    nh.param("init_y", _init_y,  0.0);
    nh.param("init_z", _init_z,  0.0);

    _odom_sub  = nh.subscribe( "odom", 1, rcvOdomCallBack );
    _odom_pub = nh.advertise<nav_msgs::Odometry>("odometry", 1);

    ros::Rate rate(200);
    bool status = ros::ok();
    while(status)
    {
        pubOdom();
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}