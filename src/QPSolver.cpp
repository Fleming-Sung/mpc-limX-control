#include "QPSolver.h"

QPSolver::QPSolver(double Ts, int N, const Eigen::MatrixXd& Ac, const Eigen::MatrixXd& Bc, 
                   const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& P,
                   const Eigen::VectorXd& x_min, const Eigen::VectorXd& x_max, 
                   double u_min, double u_max)
    : Ts(Ts), N(N), Ac(Ac), Bc(Bc), Q(Q), R(R), P(P), x_min(x_min), x_max(x_max), u_min(u_min), u_max(u_max) {
    NX = Ac.rows();
    NU = Bc.cols();

    // 初始状态
    xi = Eigen::Vector4d::Zero();
    // xi << 2, 0, 0, 0; // 初始位置
    // Eigen::Vector4d xi(2, 0, 0, 0); // 初始位置


    // 离散化系统
    discretizeSystem();
}

void QPSolver::discretizeSystem() {
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(NX + NU, NX + NU);
    M.block(0, 0, NX, NX) = Ac;
    M.block(0, NX, NX, NU) = Bc;
    
    Eigen::MatrixXd expM = (M * Ts).exp();
    Ad = expM.block(0, 0, NX, NX);
    Bd = expM.block(0, NX, NX, NU);
}

void QPSolver::buildQPParams(const Eigen::Vector4d& xi0, const Eigen::MatrixXd& xi_ref,
                             Eigen::MatrixXd& H, Eigen::VectorXd& f, Eigen::MatrixXd& A_eq, 
                             Eigen::VectorXd& b_eq, Eigen::VectorXd& lb, Eigen::VectorXd& ub, 
                             Eigen::MatrixXd& A_ineq, Eigen::VectorXd& lbA_ineq, Eigen::VectorXd& ubA_ineq) {
    // 预测矩阵构建
    Eigen::MatrixXd A_aug = Eigen::MatrixXd::Zero(NX*(N+1), NX);
    Eigen::MatrixXd B_aug = Eigen::MatrixXd::Zero(NX*(N+1), NU*N);
    A_aug.block(0,0,NX,NX) = Eigen::MatrixXd::Identity(NX, NX);
    for(int i=1; i<=N; ++i)
        A_aug.block(i*NX,0,NX,NX) = Ad * A_aug.block((i-1)*NX,0,NX,NX);
    
    B_aug.setZero();
    for(int i=1; i<=N; ++i){
        for(int j=0; j<i; ++j){
            B_aug.block(i*NX,j*NU,NX,NU) = Ad.pow(i-j-1) * Bd;
        }
    }

    // 构建QP参数
    Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(NX*(N+1), NX*(N+1));
    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(NU*N, NU*N);
    for(int i=0; i<N; ++i){
        Q_bar.block(i*NX,i*NX,NX,NX) = Q;
        R_bar.block(i*NU,i*NU,NU,NU) = R;
    }
    Q_bar.block(N*NX,N*NX,NX,NX) = P;

    H = 2*(B_aug.transpose()*Q_bar*B_aug + R_bar);
    Eigen::VectorXd xi_ref_vec = Eigen::Map<const Eigen::VectorXd>(xi_ref.data(), NX*(N+1));
    f = 2*B_aug.transpose()*Q_bar*(A_aug*xi0 - xi_ref_vec);

    // 等式约束
    A_eq = B_aug.bottomRows(NX*N);
    b_eq = A_aug.bottomRows(NX*N)*xi0;

    // 输入约束
    lb = Eigen::VectorXd::Constant(NU*N, u_min);
    ub = Eigen::VectorXd::Constant(NU*N, u_max);

    // 状态约束
    A_ineq = Eigen::MatrixXd::Zero(2*NX*N, NU*N);
    lbA_ineq = Eigen::VectorXd::Constant(2*NX*N, -qpOASES::INFTY);
    ubA_ineq = Eigen::VectorXd::Constant(2*NX*N, qpOASES::INFTY);
    for(int i=0; i<N; ++i){
        Eigen::MatrixXd A_pred = Ad.pow(i+1);
        Eigen::MatrixXd B_pred = B_aug.block((i+1)*NX,0,NX,NU*N);
        A_ineq.block(2*i*NX,0,NX,NU*N) = B_pred;
        lbA_ineq.segment(2*i*NX,NX) = x_min - A_pred*xi0;
        ubA_ineq.segment(2*i*NX,NX) = x_max - A_pred*xi0;
    }
}

bool QPSolver::solveQP(const Eigen::MatrixXd& H, const Eigen::VectorXd& f, const Eigen::MatrixXd& A_total, 
                       const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, 
                       const Eigen::VectorXd& lbA_total, const Eigen::VectorXd& ubA_total, 
                       Eigen::Matrix<double, 2, 15>& U_opt) {
    qpOASES::QProblem qp(NU*N, A_total.rows());
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;
    qp.setOptions(options);

    int nWSR = 50000;
    qpOASES::returnValue status = qp.init(H.data(), f.data(),
                                          A_total.data(), lb.data(), ub.data(),
                                          lbA_total.data(), ubA_total.data(),
                                          nWSR);

    if(status != qpOASES::SUCCESSFUL_RETURN){
        std::cerr << "QP求解失败，错误码：" << status << std::endl;
        // std::cerr << "QP求解失败，错误信息：" << qpOASES::getErrorCodeMessage(status) << std::endl;
        // return false;
    }

    qp.getPrimalSolution(U_opt.data());
    return true;
}

void QPSolver::updateState(Eigen::Vector2d& u) {
    xi = Ad * xi + Bd * u;
    std::cout << xi.transpose() << std::endl;
}

Eigen::Vector4d QPSolver::getState() {
    std::cout << xi.transpose() << std::endl;
    return xi;
}