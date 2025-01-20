/*
name: 状态估计器测试
discribe: 已完成-测试状态估计器类的功能是否实现，当前已实现虚拟状态估计器，本cpp目前仅测试虚拟状态估计器；TODO: 测试状态估计器与真值的误差对比
owner: Fleming
e-mial: sangming1006@126.com
date: Jan 20，2025
*/
// #include <ros/ros.h>
// #include <gazebo_msgs/ModelStates.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/Twist.h>
// #include <string>
// #include <iostream>
#include "state_estimator_fake.h"


int main(int argc, char **argv)
{   

    // state_estimator.init();
    ros::init(argc, argv, "state_estimator_fake_node");

    StateEstimatorFake state_estimator;

    state_estimator.spin();

    state_estimator.print();

    return 0;
}
