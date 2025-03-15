#include "QPSolver.h"
#include <cmath> // 用于数学函数
#include <Eigen/Dense>
#include "pinocchio_kinematics.h"
#include "limxsdk/datatypes.h"


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

    // 机器人状态参数
    limxsdk::RobotState Robotstate;
    Eigen::Vector3d Position;
    Eigen::Vector3d Velocity;
    Eigen::Vector3d Orientation;
    Eigen::Vector3d AngularVelocity;
    Eigen::Vector4d Quat;
    PinocchioKinematics kinematicsModel;
    int left_leg_state;
};;

mpcQP::mpcQP(limxsdk::RobotState state, Eigen::Vector3d currentPosition, Eigen::Vector3d currentVelocity, Eigen::Vector3d currentOrientation, Eigen::Vector3d currentAngularVelocity, Eigen::Vector4d currentQuat, PinocchioKinematics& kinematics, int leg) {
    // 初始化QPSolver所需的参数
    double Ts = 0.001; // 采样时间
    int N = 20; // 预测时域长度
    
    // mpcQP对象内机器人状态参数赋值
    Robotstate = state;
    Position = currentPosition;
    Velocity = currentVelocity;
    Orientation = currentOrientation;
    AngularVelocity = currentAngularVelocity;
    Quat = currentQuat;
    kinematicsModel = kinematics;
    left_leg_state = leg;    
    
    // 构建连续系统动力学模型
    buildSystemModel();

    // 设置权重和边界 TODO：摩擦条件和接触条件的边界处理
    Eigen::Matrix<double, 13, 13> Q = (Eigen::Matrix<double, 13, 1>() << 1, 1, 10, 100, 100, 100, 50, 50, 50, 100, 100, 100, 0.1).finished().asDiagonal(); // 状态权重矩阵
    Eigen::Matrix2d R = 0.1 * Eigen::Matrix3d::Identity(); // 控制输入权重矩阵
    Eigen::Matrix4d P = 20 * Q; // 终端权重矩阵
    Eigen::Vector4d x_min = (Eigen::Vector4d() << -5, -3, -5, -3).finished(); // 状态下界
    Eigen::Vector4d x_max = -x_min; // 状态上界
    double u_min = -8.0; // 控制输入下界
    double u_max = 8.0; // 控制输入上界

    // 实例化QP求解器对象
    QPSolver qpSolver(Ts, N, Ac, Bc, Q, R, P, x_min, x_max, u_min, u_max);
    
    // 设置初始状态
    Eigen::Matrix<double, 13, 1> xi;
    xi << Orientation(0),     Orientation(1),     Orientation(2), 
          Position(0),        Position(1),        Position(2),
          AngularVelocity(0), AngularVelocity(1), AngularVelocity(2),
          Velocity(0),        Velocity(1),        Velocity(2),
            -9.8;

    // 设置参考状态
    Eigen::Matrix<double, 13, 16> xi_ref; // 由于Eigen不支持以变量指定矩阵大小，所以手动指定矩阵大小，行数为状态变量维度，列数为N+1
    double omega_yaw = 0.1;
    double velocity_x = 0.5;
    for(int i=0; i<=N; ++i){
        double t = i*Ts;
        // 朝向参考(假设只能指定yaw)
        xi_ref(0,i) = Orientation(0);
        xi_ref(1,i) = Orientation(1);
        xi_ref(2,i) = Orientation(2)+t*omega_yaw;
        xi_ref(3,i) = Position(0) + t*velocity_x;
        xi_ref(4,i) = Position(1);
        xi_ref(5,i) = Position(2);
        xi_ref(6,i) = AngularVelocity(0);
        xi_ref(7,i) = AngularVelocity(1);
        xi_ref(8,i) = AngularVelocity(2);
        if (i==0) {           
            xi_ref(9,i) = Velocity(0);
        }else{
            xi_ref(9,i) = velocity_x;
        }
        xi_ref(10,i) = Velocity(1);
        xi_ref(11,i) = Velocity(2);
        xi_ref(12,i) = -9.8;
    }

    // 构建QP问题 TODO::在该类中重构该函数，使得约束条件符合实际情况
    Eigen::MatrixXd H, A_eq, A_ineq;
    Eigen::VectorXd f, b_eq, lb, ub, lbA_ineq, ubA_ineq;
    qpSolver.buildQPParams(xi, xi_ref, H, f, A_eq, b_eq, lb, ub, A_ineq, lbA_ineq, ubA_ineq);

    // 求解QP问题
    Eigen::MatrixXd A_total(A_eq.rows() + A_ineq.rows(), 2*N);
    A_total << A_eq, A_ineq;
    Eigen::VectorXd lbA_total(b_eq.size() + lbA_ineq.size());
    lbA_total << b_eq, lbA_ineq;
    Eigen::VectorXd ubA_total(b_eq.size() + ubA_ineq.size());
    ubA_total << b_eq, ubA_ineq;

    // 求解QP
    Eigen::Matrix<double, 3, 15> U_opt; // 最优控制输入矩阵
    // Eigen::MatrixXd U_opt; // 最优控制输入矩阵
    qpSolver.solveQP(H, f, A_total, lb, ub, lbA_total, ubA_total, U_opt); // TODO: 在类中显式调用QPoases求解器，适配控制输入的维度

    // 得到最优控制（所需的地面反力）
    Eigen::Vector3d u = U_opt.col(0);    
}

void mpcQP::buildSystemModel() { //TODO: 输入旋转矩阵和inB矩阵
    // 计算状态空间方程所需的参数
    // 获取当前机器人状态下的动力学模型所需参数
    // 从RobotState中获取当前关节角度
    Eigen::Quaterniond quat(Quat(0), Quat(1), Quat(2), Quat(3));
    kinematicsModel.setBaseLinkPose(Position, quat);
    std::vector<double> q(Robotstate.q.begin(), Robotstate.q.end()); // 匹配数据类型
    Eigen::VectorXd jointPositions = Eigen::Map<Eigen::VectorXd>(q.data(), q.size());
    // 基于当前关节角度计算正运动学
    kinematicsModel.forwardKinematics(jointPositions);
    // 获取当前摆动腿的末端位置
    Eigen::Vector3d footPosition;
    if (left_leg_state == 0) { // 左腿支撑
        footPosition = kinematicsModel.getLinkPosition("contact_L_Link");
    } else {                   // 右腿支撑
        footPosition = kinematicsModel.getLinkPosition("contact_R_Link");
    }
    // 计算着地点到baselink的距离
    double dx = footPosition(0) - Position(0);
    double dy = footPosition(1) - Position(1);
    double dz = footPosition(2) - Position(2);
    Eigen::Matrix3d dPos;
    dPos <<  0, dz, dy,
            dz,  0, dx,
            dy, dx,  0; // TODO: 矩阵符号有待确认
    // 地面反力对角速度的反对称矩阵，用于填入Bc
    Eigen::Matrix3d inverseInertiaMatrix = inertiaMatrix.inverse();
    Eigen::Matrix3d inB = inverseInertiaMatrix * dPos;
    // TODO:计算旋转矩阵用于填入Ac


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
}