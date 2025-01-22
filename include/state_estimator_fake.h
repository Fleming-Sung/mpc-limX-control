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
#include <tf/transform_datatypes.h>
#include <string>
#include <iostream>
#include <thread>

// 机器人状态结构体
struct RobotOdomState {
    double pos[3];  // 位置: [x, y, z]
    double ori[3];  // 姿态（欧拉角表示）: [roll, pitch, yaw](弧度)
    double quat[4]; // 姿态（四元数表示） : [x, y, z, w]
    double v_pos[3];  // 线速度: [vx, vy, vz]
    double v_ori[3];  // 角速度: [wx, wy, wz]
};

class StateEstimatorFakeNode
{
public:
    // 创建gazebo真值监听节点
    StateEstimatorFakeNode() {
        // 创建句柄
        nh_ = ros::NodeHandle();
        
        // 创建订阅者
        sub_ = nh_.subscribe("/gazebo/model_states", 10, &StateEstimatorFakeNode::modelStateCallback, this);
    }

    void spin() {
        ros::spin();
    }

    // 返回机器人状态
    RobotOdomState get_state() {
        RobotOdomState state;

        // 填充位置
        state.pos[0] = pose_.position.x;
        state.pos[1] = pose_.position.y;
        state.pos[2] = pose_.position.z;

        /// 将四元数转换为欧拉角
        tf::Quaternion quat(
            pose_.orientation.x,
            pose_.orientation.y,
            pose_.orientation.z,
            pose_.orientation.w
        );
        
        tf::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);  // 获取 roll, pitch, yaw（单位：弧度）

        // 填充姿态（欧拉角）
        state.ori[0] = roll;   // roll (绕x轴的旋转)
        state.ori[1] = pitch;  // pitch (绕y轴的旋转)
        state.ori[2] = yaw;    // yaw (绕z轴的旋转)

        state.quat[0] = pose_.orientation.x;
        state.quat[1] = pose_.orientation.y;
        state.quat[2] = pose_.orientation.z;
        state.quat[3] = pose_.orientation.w;

        // 填充线速度
        state.v_pos[0] = twist_.linear.x;
        state.v_pos[1] = twist_.linear.y;
        state.v_pos[2] = twist_.linear.z;

        // 填充角速度
        state.v_ori[0] = twist_.angular.x;
        state.v_ori[1] = twist_.angular.y;
        state.v_ori[2] = twist_.angular.z;

        return state;
    }

    // // 从节点获取机器人状态的接口
    // void getState(std::string &robot_name, geometry_msgs::Pose &robot_pose, geometry_msgs::Twist &robot_twist) const {
    //     robot_name = name_;
    //     robot_pose = pose_;
    //     robot_twist = twist_;
    // }

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

    // 构造函数
    StateEstimatorFake() {}

    void spin() {
        ros::Rate loop_rate(10);  // 设置循环频率（10Hz）

        while (ros::ok()) {
            ros::spinOnce();  // 处理ROS消息

            loop_rate.sleep();  // 保证循环频率
        }
    }

    RobotOdomState get_state() {
        return state_estimator_node.get_state();
    }

    // // 定期打印状态
    // void print() {
    //     state_estimator_node.getState(robot_name, robot_pose, robot_twist);

    //     ROS_INFO("Robot %s: Position(%.2f, %.2f, %.2f), Orientation(%.2f, %.2f, %.2f), Twist(%.2f, %.2f, %.2f)",
    //             robot_name.c_str(),
    //             robot_pose.position.x, robot_pose.position.y, robot_pose.position.z,
    //             robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z,
    //             robot_twist.linear.x, robot_twist.linear.y, robot_twist.linear.z);
    // }   

    // // 外部调用的打印状态方法
    // void printStateOnce() {
    //     print();  // 调用内部的print方法来打印当前状态
    // }
};
