#ifndef QP_SOLVER_H
#define QP_SOLVER_H

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <unsupported/Eigen/MatrixFunctions>

class QPSolver {
public:
    // 构造函数
    QPSolver(double Ts, int N, const Eigen::MatrixXd& Ac, const Eigen::MatrixXd& Bc, 
             const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& P,
             const Eigen::VectorXd& x_min, const Eigen::VectorXd& x_max, 
             double u_min, double u_max);

    // 离散化系统
    void discretizeSystem();

    // 构建QP参数
    void buildQPParams(const Eigen::Vector4d& xi0, const Eigen::MatrixXd& xi_ref,
                       Eigen::MatrixXd& H, Eigen::VectorXd& f, Eigen::MatrixXd& A_eq, 
                       Eigen::VectorXd& b_eq, Eigen::VectorXd& lb, Eigen::VectorXd& ub, 
                       Eigen::MatrixXd& A_ineq, Eigen::VectorXd& lbA_ineq, Eigen::VectorXd& ubA_ineq);

    // 求解QP问题
    bool solveQP(const Eigen::MatrixXd& H, const Eigen::VectorXd& f, const Eigen::MatrixXd& A_total, 
                 const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, 
                 const Eigen::VectorXd& lbA_total, const Eigen::VectorXd& ubA_total, 
                 Eigen::Matrix<double, 2, 15>& U_opt);

    // 更新状态
    void updateState(Eigen::Vector2d& u);

    // 获取当前状态
    Eigen::Vector4d getState();

private:
    double Ts;
    int N;
    int NX;
    int NU;
    Eigen::MatrixXd Ac;
    Eigen::MatrixXd Bc;
    Eigen::MatrixXd Ad;
    Eigen::MatrixXd Bd;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd P;
    Eigen::VectorXd x_min;
    Eigen::VectorXd x_max;
    double u_min;
    double u_max;
    Eigen::Vector4d xi;
};

#endif // QP_SOLVER_H