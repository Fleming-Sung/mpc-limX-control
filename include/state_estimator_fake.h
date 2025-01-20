/*
name: 虚拟状态估计器
discribe: 直接从gazebo中读取话题真值作为状态估计器，用于辅助MPC机器人locomotino的开发
owner: Fleming
e-mial: sangming1006@126.com
date: Jan 20，2025
进度：能编译通过，没有显示bug；但功能还不完整，卡在spin的逻辑上，参加gpt解释。
*/
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <iostream>

class StateEstimatorFakeNode
{
public:
    // 创建gazebo真值监听节点
    StateEstimatorFakeNode() {
        // // // 确保只初始化一次节点
        // if (!ros::isInitialized()) {
        //     ros::init(argc, argv, "state_estimator_fake_node"); // 初始化ROS节点
        // }

        // 创建句柄
        nh_ = ros::NodeHandle();
        
        // 创建订阅者
        sub_ = nh_.subscribe("/gazebo/model_states", 10, &StateEstimatorFakeNode::modelStateCallback, this);
    }

    void spin() {
        ros::spin();
    }

    // 从节点获取机器人状态的接口
    void getState(std::string &robot_name, geometry_msgs::Pose &robot_pose, geometry_msgs::Twist &robot_twist) const {
        robot_name = name_;
        robot_pose = pose_;
        robot_twist = twist_;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    std::string name_;
    geometry_msgs::Pose pose_;
    geometry_msgs::Twist twist_;

    // 回调函数：处理接收到的消息并更新内部状态
    void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        // 遍历所有模型，找到相应机器人的消息
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "point_foot_robot") { // 需要读取的机器人的名称
                // 更新状态
                name_ = msg->name[i];
                pose_ = msg->pose[i];
                twist_ = msg->twist[i];
                break;
            }
        }
    }
};

class StateEstimatorFake
{
public:
    // 获取状态来自仿真节点的机器人状态信息
    std::string robot_name;
    geometry_msgs::Pose robot_pose;
    geometry_msgs::Twist robot_twist;
    StateEstimatorFakeNode state_estimator_node; 

    // // StateEstimatorFakeNode 对象作为 StateEstimatorFake 类的成员变量
    // StateEstimatorFake(int argc, char** argv) 
    //     : state_estimator_node(argc, argv){

    //     }

    void spin() {
        state_estimator_node.spin();
    }

    void print() {
        state_estimator_node.getState(robot_name, robot_pose, robot_twist);

        ROS_INFO("Robot %s: Position(%.2f, %.2f, %.2f), Orientation(%.2f, %.2f, %.2f), Twist(%.2f, %.2f, %.2f)",
                robot_name.c_str(),
                robot_pose.position.x, robot_pose.position.y, robot_pose.position.z,
                robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z,
                robot_twist.linear.x, robot_twist.linear.y, robot_twist.linear.z);
    }   
    
};




// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "robot_state_tracker_cpp");
//     ros::NodeHandle nh;

//     // 订阅 /gazebo/model_states 话题
//     ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 10, modelStateCallback);

//     // 进入循环，持续获取话题数据
//     ros::spin();

//     return 0;
// }
