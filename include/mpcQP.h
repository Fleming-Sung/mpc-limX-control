#include "QPSolver.h"
#include <cmath> // 用于数学函数
#include <Eigen/Dense>
#include "pinocchio_kinematics.h"

class mpcQP{
public:
    mpcQP(limxsdk::RobotState state, Eigen::Vector3d currentPosition, Eigen::Vector3d currentVelocity, Eigen::Vector3d currentOrientation, Eigen::Vector3d currentAngularVelocity, Eigen::Vector4d currentQuat, PinocchioKinematics& kinematics, int leg); // 构造函数
    void buildSystemModel(); // 构建系统模型
    // void run();
private:
    Eigen::Matrix<double, 13, 13> Ac = Eigen::Matrix<double, 13, 13>::Zero(); // 连续时间系统矩阵
    Eigen::Matrix<double, 13, 3> Bc = Eigen::Matrix<double, 13, 3>::Zero(); // 连续时间输入矩阵
    
    // 机器人本体重量参数
    double m = 9.585; // 机器人本体质量
    // 机器人本体惯性矩阵
    Eigen::Matrix3d inertiaMatrix = (Eigen::Matrix3d() << 140110.479E-06,    534.939E-06,  28184.116E-06,
                                                             534.939E-06, 110641.449E-06,    -27.278E-06,
                                                           28184.116E-06,    -27.278E-06,  98944.542E-06).finished();
};;

mpcQP::mpcQP(limxsdk::RobotState state, Eigen::Vector3d currentPosition, Eigen::Vector3d currentVelocity, Eigen::Vector3d currentOrientation, Eigen::Vector3d currentAngularVelocity, Eigen::Vector4d currentQuat, PinocchioKinematics& kinematics, int left_leg_state) {
    // 初始化QPSolver
    double Ts = 0.01; // 采样时间
    int N = 20; // 预测时域长度

    // 从RobotState中获取当前关节角度
    Eigen::Quaterniond quat(currentQuat(0), currentQuat(1), currentQuat(2), currentQuat(3));
    kinematics.setBaseLinkPose(currentPosition, quat);
    std::vector<double> q(state.q.begin(), state.q.end()); // 匹配数据类型
    Eigen::VectorXd jointPositions = Eigen::Map<Eigen::VectorXd>(q.data(), q.size());
    // 基于当前关节角度计算正运动学
    kinematics.forwardKinematics(jointPositions);
    // 获取当前摆动腿的末端位置
    Eigen::Vector3d footPosition;
    if (left_leg_state == 0) { // 左腿支撑
        footPosition = kinematics.getLinkPosition("contact_L_Link");
    } else {                   // 右腿支撑
        footPosition = kinematics.getLinkPosition("contact_R_Link");
    }

    // 计算着地点到baselink的距离
    double dx = footPosition(0) - currentPosition(0);
    double dy = footPosition(1) - currentPosition(1);
    double dz = footPosition(2) - currentPosition(2);
    Eigen::Matrix3d dPos;
    dPos <<  0, dz, dy,
            dz,  0, dx,
            dy, dx,  0;
    // 地面反力对角速度的乘子矩阵，用于填入Bc
    Eigen::Matrix3d inverseInertiaMatrix = inertiaMatrix.inverse();
    Eigen::Matrix3d inB = inverseInertiaMatrix * dPos;
    
    Eigen::Matrix<double, 13, 13> Q = (Eigen::Matrix<double, 13, 1>() << 1, 1, 10, 100, 100, 100, 50, 50, 50, 100, 100, 100, 0.1).finished().asDiagonal(); // 状态权重矩阵
    Eigen::Matrix2d R = 0.1 * Eigen::Matrix3d::Identity(); // 控制输入权重矩阵
    Eigen::Matrix4d P = 20 * Q; // 终端权重矩阵
    Eigen::Vector4d x_min = (Eigen::Vector4d() << -5, -3, -5, -3).finished(); // 状态下界
    Eigen::Vector4d x_max = -x_min; // 状态上界
    double u_min = -8.0; // 控制输入下界
    double u_max = 8.0; // 控制输入上界

    QPSolver qpSolver(Ts, N, Ac, Bc, Q, R, P, x_min, x_max, u_min, u_max);
}

void mpcQP::buildSystemModel() {
    // 计算状态空间方程所需的参数


    // 填入系统连续时间状态模型矩阵
    //  col1, col2, col3, col4, col5, col6, col7, col8, col9, col10, col11, col12, col13
    Ac << 0,    0,    0,    0,    0,    0,    0,   dz,   dy,    0,     0,     0,     0,    // row 1
          0,    0,    0,    0,    0,    0,   dz,    0,   dx,    0,     0,     0,     0,    // row 2
          0,    0,    0,    0,    0,    0,   dy,   dx,    0,    0,     0,     0,     0,    // row 3
          0,    0,    0,    0,    0,    0,    0,    0,    0,    1,     0,     0,     0,    // row 4
          0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     1,     0,     0,    // row 5
          0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     1,     0,    // row 6
          0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    // row 7
          0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    // row 8
          0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    // row 9
          0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    // row 10
          0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0,    // row 11
          0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,    -1,    // row 12
          0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0;    // row 13

    //  col1, col2, col3
    Bc << 0,    0,    0,    // row 1
          0,    0,    0,    // row 2
          0,    0,    0,    // row 3
          0,    0,    0,    // row 4
          0,    0,    0,    // row 5
          0,    0,    0,    // row 6
          0,    0,    0,    // row 7
          0,    0,    0,    // row 8
          0,    0,    0,    // row 9
         -m,    0,    0,    // row 10
          0,   -m,    0,    // row 11
          0,    0,   -m,    // row 12
          0,    0,    0;    // row 13





    Eigen::Matrix4d Q = (Eigen::Vector4d() << 50, 5, 50, 5).finished().asDiagonal(); // 状态权重矩阵
    Eigen::Matrix2d R = 0.1 * Eigen::Matrix2d::Identity(); // 控制输入权重矩阵
    Eigen::Matrix4d P = 20 * Q; // 终端权重矩阵
    Eigen::Vector4d x_min = (Eigen::Vector4d() << -5, -3, -5, -3).finished(); // 状态下界
    Eigen::Vector4d x_max = -x_min; // 状态上界
    double u_min = -8.0; // 控制输入下界
    double u_max = 8.0; // 控制输入上界

    QPSolver qpSolver(Ts, N, Ac, Bc, Q, R, P, x_min, x_max, u_min, u_max);

    // 初始状态
    Eigen::Vector4d xi(2, 0, 0, 0); // 初始位置(0.5,-0.5)，速度为零

    // 参考轨迹参数
    double trajectory_radius = 2.0; // 轨迹半径
    double angular_vel = 0.5; // 角速度，单位：rad/s

    // 最优控制的初值
    Eigen::Vector2d u(0, 0);

    for(int k=0; k<500; ++k){
        // 生成动态圆形参考轨迹
        Eigen::Matrix<double, 4, 16> xi_ref; // 参考轨迹矩阵
        for(int i=0; i<=N; ++i){
            double t = k*Ts + i*Ts;
            double theta = angular_vel * t;
            // 位置参考
            xi_ref(0,i) = trajectory_radius * cos(theta);
            xi_ref(2,i) = trajectory_radius * sin(theta);
            // 速度参考（导数计算）
            xi_ref(1,i) = -trajectory_radius * angular_vel * sin(theta);
            xi_ref(3,i) = trajectory_radius * angular_vel * cos(theta);
        }

        // 构建QP问题
        Eigen::MatrixXd H, A_eq, A_ineq;
        Eigen::VectorXd f, b_eq, lb, ub, lbA_ineq, ubA_ineq;
        qpSolver.buildQPParams(xi, xi_ref, H, f, A_eq, b_eq, lb, ub, A_ineq, lbA_ineq, ubA_ineq);

        // 合并约束
        Eigen::MatrixXd A_total(A_eq.rows() + A_ineq.rows(), 2*N);
        A_total << A_eq, A_ineq;
        Eigen::VectorXd lbA_total(b_eq.size() + lbA_ineq.size());
        lbA_total << b_eq, lbA_ineq;
        Eigen::VectorXd ubA_total(b_eq.size() + ubA_ineq.size());
        ubA_total << b_eq, ubA_ineq;

        // 求解QP
        Eigen::Matrix<double, 2, 15> U_opt; // 最优控制输入矩阵
        // Eigen::MatrixXd U_opt; // 最优控制输入矩阵
        if (!qpSolver.solveQP(H, f, A_total, lb, ub, lbA_total, ubA_total, U_opt)) {
            return -1;
        }

        // 应用控制
        Eigen::Vector2d u = U_opt.col(0);
        qpSolver.updateState(u);
        xi = qpSolver.getState();
        std::cout << xi.transpose() << std::endl;

        // 可视化输出
        Eigen::Vector2d pos;
        pos << xi.transpose()[0], xi.transpose()[2];
        Eigen::Vector2d ref_pos(xi_ref(0,0), xi_ref(2,0));
        double error = (pos - ref_pos).norm();
        
        std::cout << "\n=== 时间步 " << k << " ==="
                  << "\n控制量: [" << u.transpose() << "]"
                  << "\n实际位置: (" << pos.transpose() << ")"
                  << "\n参考位置: (" << ref_pos.transpose() << ")"
                  << "\n跟踪误差: " << error 
                  << "\n状态: [" << qpSolver.getState().transpose() << "]" << std::endl;
    }
}