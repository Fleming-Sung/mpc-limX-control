/*
name: 虚拟状态估计器
discribe: 直接从gazebo中读取话题真值作为状态估计器，用于辅助MPC机器人locomotino的开发
owner: Fleming
e-mial: sangming1006@126.com
date: Jan 20，2025
*/
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <iostream>

class RobotState
{
public:
    std::string name;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;

    RobotState() : name(""), pose(), twist() {}

    void update(const std::string &robot_name, const geometry_msgs::Pose &robot_pose, const geometry_msgs::Twist &robot_twist)
    {
        name = robot_name;
        pose = robot_pose;
        twist = robot_twist;
    }

    void print() const
    {
        ROS_INFO("Robot %s: Position(%.2f, %.2f, %.2f), Orientation(%.2f, %.2f, %.2f), Twist(%.2f, %.2f, %.2f)",
                 name.c_str(),
                 pose.position.x, pose.position.y, pose.position.z,
                 pose.orientation.x, pose.orientation.y, pose.orientation.z,
                 twist.linear.x, twist.linear.y, twist.linear.z);
    }
};

void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    RobotState robot;

    // 遍历所有模型，找到你的机器人
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == "point_foot_robot") // 需要读取的机器人的名称
        {
            robot.update(msg->name[i], msg->pose[i], msg->twist[i]);
            robot.print();
            break;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_state_tracker_cpp");
    ros::NodeHandle nh;

    // 订阅 /gazebo/model_states 话题
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 10, modelStateCallback);

    // 进入循环，持续获取话题数据
    ros::spin();

    return 0;
}
