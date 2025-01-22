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


int main(int argc, char **argv) {
    ros::init(argc, argv, "state_estimator_fake_node");
    ros::NodeHandle nh;

    StateEstimatorFake state_estimator;

    // 创建一个定时器，每 0.5 秒（2Hz）调用一次
    ros::Rate loop_rate(2);  // 2Hz

    while (ros::ok()) {
        // 获取当前的机器人状态
        RobotOdomState state = state_estimator.get_state();

        // 打印机器人状态
        std::cout << "Robot state:" << std::endl;
        std::cout << "Position: [" << state.pos[0] << ", " << state.pos[1] << ", " << state.pos[2] << "]" << std::endl;
        std::cout << "Orientation (Euler): [" << state.ori[0] << ", " << state.ori[1] << ", " << state.ori[2] << "]" << std::endl;
        std::cout << "Linear Velocity: [" << state.v_pos[0] << ", " << state.v_pos[1] << ", " << state.v_pos[2] << "]" << std::endl;
        std::cout << "Angular Velocity: [" << state.v_ori[0] << ", " << state.v_ori[1] << ", " << state.v_ori[2] << "]" << std::endl;

        // 休眠直到下一个周期
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
