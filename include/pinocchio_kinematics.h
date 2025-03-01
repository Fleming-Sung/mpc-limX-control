#ifndef PINOCCHIO_KINEMATICS_H
#define PINOCCHIO_KINEMATICS_H

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <iostream>
#include <chrono>
#include <Eigen/Dense>

class PinocchioKinematics {
public:
    // 实例化pinocchio所需的模型和数据
    pinocchio::Model model;
    pinocchio::Data data;
    

    // 基于URDF文件构建PinocchioKinematics类
    PinocchioKinematics() {
        std::string urdf_path = "./src/robot-description/pointfoot/PF_TRON1A/urdf/robot.urdf";
        pinocchio::urdf::buildModel(urdf_path, model);
        data = pinocchio::Data(model);
    }

    // 前向运动学计算
    void forwardKinematics(const Eigen::VectorXd& q) {
        pinocchio::forwardKinematics(model, data, q);   // 计算正运动学
        pinocchio::updateFramePlacements(model, data);  // 更新所有frame的位置，更新后才能查询静态link的位置
    }

    // 输入link名称，返回link的位置
    Eigen::Vector3d getLinkPosition(const std::string& link_name) {
        pinocchio::FrameIndex frame_id = model.getFrameId(link_name);
        if (frame_id >= model.nframes) {
            std::cerr << "Cannot find frame: " << link_name << " in the model." << std::endl;
            return Eigen::Vector3d::Zero();
        }
        return data.oMf[frame_id].translation();
    }

    // 基于牛顿-欧拉法计算逆动力学
    Eigen::VectorXd inverseDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, const Eigen::VectorXd& ddq) {
        return pinocchio::rnea(model, data, q, dq, ddq);
    }

    // 输入一组关节角度，打印所有关节的位置
    void printJointPositions(const Eigen::VectorXd& q) {
        forwardKinematics(q);
        std::cout << "Joint positions:" << std::endl;
        for (size_t i = 0; i < model.njoints; ++i) {
            std::cout << "Joint " << model.names[i] << ": " 
                      << data.oMi[i].translation().transpose() << std::endl;
        }
    }

    // 逆运动学计算函数
    Eigen::VectorXd inverseKinematics(const std::string& link_name, const Eigen::Vector3d& target_position, const Eigen::VectorXd& initial_guess, double tolerance = 1e-3, int max_iterations = 10) {
        // 初始化关节角度为初始猜测值
        Eigen::VectorXd q = initial_guess;
        // 获取指定link的frame ID
        pinocchio::FrameIndex frame_id = model.getFrameId(link_name);
        // 检查frame ID是否有效
        if (frame_id >= model.nframes) {
            std::cerr << "Cannot find frame: " << link_name << " in the model." << std::endl;
            return q;
        }

        // 设定容差、最大迭代次数、时间步长和阻尼系数
        const double eps = tolerance;
        const int IT_MAX = max_iterations;
        const double DT = 1e-1;
        const double damp = 1e-6;

        // 初始化雅可比矩阵
        pinocchio::Data::Matrix6x J(6, model.nv);
        J.setZero();

        // 初始化成功标志
        bool success = false;
        // 定义误差向量
        typedef Eigen::Matrix<double, 6, 1> Vector6d;
        Vector6d err;
        // 定义速度向量
        Eigen::VectorXd v(model.nv);

        // 目标位姿
        pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), target_position);

        // 迭代求解
        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < IT_MAX; ++i) {
            // 打印每轮循环花费的系统时间
            auto now = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start);
            std::cout << "Iteration " << i << " took " << duration.count() << " microseconds." << std::endl;
            start = now;
            

            // 计算正运动学
            pinocchio::forwardKinematics(model, data, q);
            // 计算当前位姿与目标位姿的误差
            const pinocchio::SE3 iMd = data.oMf[frame_id].actInv(oMdes);
            err = pinocchio::log6(iMd).toVector(); // 在关节坐标系下的误差

            // 检查误差是否在容差范围内
            if (err.norm() < eps) {
                success = true;
                break;
            }

            // 计算雅可比矩阵
            pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J); // 在关节坐标系下的雅可比矩阵
            pinocchio::Data::Matrix6 Jlog;
            pinocchio::Jlog6(iMd.inverse(), Jlog);
            J = -Jlog * J;
            // 计算JJ^T矩阵并添加阻尼项
            pinocchio::Data::Matrix6 JJt;
            JJt.noalias() = J * J.transpose();
            JJt.diagonal().array() += damp;
            // 计算速度向量
            v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
            // 更新关节角度
            q = pinocchio::integrate(model, q, v * DT);
            

            // // 每10次迭代打印一次误差
            // if (!(i % 10)) {
            //     std::cout << i << ": error = " << err.transpose() << std::endl;
            // }
        }

        // 打印收敛信息
        if (success) {
            std::cout << "Convergence achieved!" << std::endl;
        } else {
            std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
        }

        // 打印最终结果和误差
        std::cout << "\nresult: " << q.transpose() << std::endl;
        std::cout << "\nfinal error: " << err.transpose() << std::endl;

        // 返回最终的关节角度
        return q;
    }


    // 设定base_link的位姿
    void setBaseLinkPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
        pinocchio::SE3 base_pose(orientation.toRotationMatrix(), position);
        data.oMi[1] = base_pose; // Assuming base_link is the first joint
        pinocchio::updateFramePlacements(model, data);
    }

private:
    
};

#endif // PINOCCHIO_KINEMATICS_H