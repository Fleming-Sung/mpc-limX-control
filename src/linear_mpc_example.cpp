#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <unsupported/Eigen/MatrixFunctions>

typedef Eigen::MatrixXd MatrixXd;
typedef Eigen::VectorXd VectorXd;
typedef Eigen::Matrix4d Matrix4d;
typedef Eigen::Matrix<double, 4, 2> Matrix4x2d;

const int N = 15;       // 延长预测时域
const int NX = 4;
const int NU = 2;

// 系统参数
double damping = 0.02;
double mass = 0.2;
Matrix4d Ac;
Matrix4x2d Bc;
Matrix4d Ad;
Matrix4x2d Bd;

// 权重矩阵（增强状态跟踪权重）
Matrix4d Q = (Eigen::Vector4d() << 50, 5, 50, 5).finished().asDiagonal();
Eigen::Matrix2d R = 0.1*Eigen::Matrix2d::Identity();
Matrix4d P = 20*Q;

// 约束条件
double u_min = -8.0, u_max = 8.0;
Eigen::Vector4d x_min = (Eigen::Vector4d() << -5, -3, -5, -3).finished();
Eigen::Vector4d x_max = -x_min;

// 精确离散化
void discretizeSystem(double Ts) {
    Matrix4d A = Ac;
    Ad = (A * Ts).exp();
    
    // 数值积分计算Bd
    int steps = 100;
    Bd.setZero();
    for(int i=0; i<steps; ++i){
        double tau = i*Ts/steps;
        Bd += Ad * (Matrix4d::Identity() - A*tau/steps).inverse() * Bc * (Ts/steps);
    }
}

void buildQPParams(const Eigen::Vector4d& xi0,
                   const Eigen::Matrix<double, NX, N+1>& xi_ref,
                   MatrixXd& H,
                   VectorXd& f,
                   MatrixXd& A_eq,
                   VectorXd& b_eq,
                   VectorXd& lb,
                   VectorXd& ub,
                   MatrixXd& A_ineq,
                   VectorXd& lbA_ineq,
                   VectorXd& ubA_ineq) {
    
    // 预测矩阵构建
    MatrixXd A_aug(NX*(N+1), NX);
    MatrixXd B_aug(NX*(N+1), NU*N);
    A_aug.block(0,0,NX,NX) = Matrix4d::Identity();
    for(int i=1; i<=N; ++i)
        A_aug.block(i*NX,0,NX,NX) = Ad * A_aug.block((i-1)*NX,0,NX,NX);
    
    B_aug.setZero();
    for(int i=1; i<=N; ++i){
        for(int j=0; j<i; ++j){
            B_aug.block(i*NX,j*NU,NX,NU) = Ad.pow(i-j-1) * Bd;
        }
    }

    // 构建QP参数
    MatrixXd Q_bar = MatrixXd::Zero(NX*(N+1), NX*(N+1));
    MatrixXd R_bar = MatrixXd::Zero(NU*N, NU*N);
    for(int i=0; i<N; ++i){
        Q_bar.block(i*NX,i*NX,NX,NX) = Q;
        R_bar.block(i*NU,i*NU,NU,NU) = R;
    }
    Q_bar.block(N*NX,N*NX,NX,NX) = P;

    H = 2*(B_aug.transpose()*Q_bar*B_aug + R_bar);
    Eigen::VectorXd xi_ref_vec = Eigen::Map<const VectorXd>(xi_ref.data(), NX*(N+1));
    f = 2*B_aug.transpose()*Q_bar*(A_aug*xi0 - xi_ref_vec);

    // 等式约束
    A_eq = B_aug.bottomRows(NX*N);
    b_eq = A_aug.bottomRows(NX*N)*xi0;

    // 输入约束
    lb = VectorXd::Constant(NU*N, u_min);
    ub = VectorXd::Constant(NU*N, u_max);

    // 状态约束
    A_ineq = MatrixXd::Zero(2*NX*N, NU*N);
    lbA_ineq = VectorXd::Constant(2*NX*N, -qpOASES::INFTY);
    ubA_ineq = VectorXd::Constant(2*NX*N, qpOASES::INFTY);
    for(int i=0; i<N; ++i){
        MatrixXd A_pred = Ad.pow(i+1);
        MatrixXd B_pred = B_aug.block((i+1)*NX,0,NX,NU*N);
        A_ineq.block(2*i*NX,0,NX,NU*N) = B_pred;
        lbA_ineq.segment(2*i*NX,NX) = x_min - A_pred*xi0;
        ubA_ineq.segment(2*i*NX,NX) = x_max - A_pred*xi0;
    }
}

int main() {
    // 系统建模
    Ac << 0,1,0,0,
          0,-damping/mass,0,0,
          0,0,0,1,
          0,0,0,-damping/mass;
    Bc << 0,0,
          1/mass,0,
          0,0,
          0,1/mass;

    // 离散化参数
    double Ts = 0.01;
    discretizeSystem(Ts);

    // 初始状态（偏离参考轨迹）
    Eigen::Vector4d xi(2, 0, 0, 0); // 初始位置(0.5,-0.5)，速度为零

    // 参考轨迹参数
    double trajectory_radius = 2.0;
    double angular_vel = 0.5; // rad/s

    // 最优控制的初值
    Eigen::Vector2d u(0, 0);

    for(int k=0; k<500; ++k){
        // 生成动态圆形参考轨迹
        Eigen::Matrix<double, NX, N+1> xi_ref;
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
        MatrixXd H, A_eq, A_ineq;
        VectorXd f, b_eq, lb, ub, lbA_ineq, ubA_ineq;
        buildQPParams(xi, xi_ref, H, f, A_eq, b_eq, lb, ub, A_ineq, lbA_ineq, ubA_ineq);

        // 合并约束
        MatrixXd A_total(A_eq.rows() + A_ineq.rows(), NU*N);
        A_total << A_eq, A_ineq;
        VectorXd lbA_total(b_eq.size() + lbA_ineq.size());
        lbA_total << b_eq, lbA_ineq;
        VectorXd ubA_total(b_eq.size() + ubA_ineq.size());
        ubA_total << b_eq, ubA_ineq;

        // 求解QP
        qpOASES::QProblem qp(NU*N, A_total.rows());
        qpOASES::Options options;
        options.printLevel = qpOASES::PL_NONE;
        qp.setOptions(options);

        int nWSR = 50000;
        qpOASES::returnValue status = qp.init(H.data(), f.data(),
                                             A_total.data(), lb.data(), ub.data(),
                                             lbA_total.data(), ubA_total.data(),
                                             nWSR);

        // if(status != qpOASES::SUCCESSFUL_RETURN){
        //     std::cerr << "QP求解失败，错误码：" << status << std::endl;
        //     return -1;
        // }

        // 应用控制
        Eigen::Matrix<double, NU, N> U_opt;
        qp.getPrimalSolution(U_opt.data());
        Eigen::Vector2d du = U_opt.col(0);
        u = du;
        xi = Ad * xi + Bd * u;

        // 可视化输出
        Eigen::Vector2d pos(xi[0], xi[2]);
        Eigen::Vector2d ref_pos(xi_ref(0,0), xi_ref(2,0));
        double error = (pos - ref_pos).norm();
        
        std::cout << "\n=== 时间步 " << k << " ==="
                  << "\n控制量: [" << u.transpose() << "]"
                  << "\n实际位置: (" << pos.transpose() << ")"
                  << "\n参考位置: (" << ref_pos.transpose() << ")"
                  << "\n跟踪误差: " << error 
                  << "\n状态: [" << xi.transpose() << "]" << std::endl;
    }
    return 0;
}