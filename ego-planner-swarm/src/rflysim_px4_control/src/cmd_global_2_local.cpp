#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "quadrotor_msgs/PositionCommand.h"

ros::Subscriber _cmd_sub;
ros::Publisher  _cmd_pub;

//quadrotor_msgs::PositionCommand _cmd;
quadrotor_msgs::PositionCommand _cmd;
double _init_x, _init_y, _init_z;

bool rcv_cmd = false;
void rcvPosCmdCallBack(const quadrotor_msgs::PositionCommand cmd)
{
    rcv_cmd = true;
    _cmd    = cmd;
}
void pubCmd()
{
    quadrotor_msgs::PositionCommand cmd = _cmd;
    cmd.header.stamp    = ros::Time::now();

    if(rcv_cmd)
    {
        cmd.position.x -= _init_x;
        cmd.position.y -= _init_y;
        cmd.position.z -= _init_z;
        _cmd_pub.publish(cmd);
    }

}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "cmd_generator");
    ros::NodeHandle nh( "~" );

    nh.param("init_x", _init_x,  0.0);
    nh.param("init_y", _init_y,  0.0);
    nh.param("init_z", _init_z,  0.0);

    _cmd_sub  = nh.subscribe( "cmd", 1, rcvPosCmdCallBack );
    _cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("command", 1);

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status)
    {
        pubCmd();
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}