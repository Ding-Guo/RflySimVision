#!/bin/bash

# 启动 swarm 脚本
cd /home/sensor_pkg
python swarm.py &
sleep 1

# 启动 Rflysim PX4 节点
cd /home/ego-planner-swarm
source devel/setup.bash
roslaunch rflysim_px4_node rflysim_px4_three_udp.launch &
if [ $? -ne 0 ]; then
    echo "启动 rflysim_px4_three_udp 失败"
    exit 1
fi

sleep 15

# 启动控制节点
roslaunch px4ctrl run_ctrl_rflsyim_three.launch &
if [ $? -ne 0 ]; then
    echo "启动 run_ctrl_rflsyim_three 失败"
    exit 1
fi

sleep 3

# 启动 RViz
roslaunch ego_planner rviz.launch &
if [ $? -ne 0 ]; then
    echo "启动 RViz 失败"
    exit 1
fi

sleep 10

# 起飞命令
rostopic pub -1 /px4ctrl_uav0/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
rostopic pub -1 /px4ctrl_uav1/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
rostopic pub -1 /px4ctrl_uav2/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"

sleep 5

# 启动三个无人机的仿真
roslaunch ego_planner rflysim_three_drone.launch
if [ $? -ne 0 ]; then
    echo "启动 rflysim_three_drone 失败"
    exit 1
fi