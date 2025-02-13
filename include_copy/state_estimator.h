#include <vector>
#include <iostream>
#include <memory>
#include <Eigen/Dense>

// IMU数据结构
struct ImuData {
    uint64_t stamp;  // 时间戳，单位是纳秒，表示数据记录的时间
    float acc[3];    // IMU加速度，包括X、Y、Z轴
    float gyro[3];   // IMU降速仪（角速度），包括X、Y、Z轴
    float quat[4];   // IMU四元数，表示方向（w， x， y， z）
};

typedef std::shared_ptr<ImuData> ImuDataPtr;
typedef std::shared_ptr<const ImuData> ImuDataConstPtr;

// 机器人状态结构
struct RobotState {
    uint64_t stamp;            // 时间戳，单位是纳秒，表示数据记录的时间
    std::vector<float> tau;    // 当前轮活转矩，单位为牛米（Nm）
    std::vector<float> q;      // 当前关节角，单位为弧度
    std::vector<float> dq;     // 当前关节速度，单位为弧速（rad/s）

    RobotState(int motor_num = 3) : tau(motor_num, 0.0), q(motor_num, 0.0), dq(motor_num, 0.0) {}
};

typedef std::shared_ptr<RobotState> RobotStatePtr;
typedef std::shared_ptr<const RobotState> RobotStateConstPtr;

// 状态观测器类
class StateEstimator {
public:
    StateEstimator() : v_b(Eigen::Vector3f::Zero()), dt(0.001), g(9.81) {
        Q = Eigen::Matrix3f::Identity() * 0.01; // 过程器出误协方带
        R = Eigen::Matrix3f::Identity() * 0.1;  // 观测器器出误协方带
        P = Eigen::Matrix3f::Identity();        // 初始协方带
    }

    // 初始化功能，设置初始速度
    void initialize(const Eigen::Vector3f& initial_velocity) {
        v_b = initial_velocity;
        P.setIdentity();
    }

    // 状态预测，基于IMU加速度
    void predict(const ImuDataConstPtr& imu) {
        Eigen::Matrix3f R_b = quaternionToRotationMatrix(imu->quat);
        Eigen::Vector3f acc(imu->acc[0], imu->acc[1], imu->acc[2]);
        Eigen::Vector3f gravity(0, 0, -g);

        // 预测速度
        v_b = v_b + dt * (R_b * acc + gravity);

        // 更新协方带：P = F * P * F^T + Q
        Eigen::Matrix3f F = Eigen::Matrix3f::Identity(); // 状态转移矩阵
        P = F * P * F.transpose() + Q;
    }  

    // 状态更新，基于关节规则
    void update(const RobotStateConstPtr& robot_state, int leg) {
        Eigen::Vector3f dq(robot_state->dq[0], robot_state->dq[1], robot_state->dq[2]);
        Eigen::Matrix3f J = computeJacobian(robot_state->q, leg);

        // 计算观测值（基于关节规则）
        Eigen::Vector3f v_obs = J * dq;

        // 卡尔曼增益：K = P * H^T * (H * P * H^T + R)^(-1)
        Eigen::Matrix3f H = Eigen::Matrix3f::Identity(); // 观测矩阵
        Eigen::Matrix3f S = H * P * H.transpose() + R;  // 列联协方带
        Eigen::Matrix3f K = P * H.transpose() * S.inverse();

        // 更新状态：X = X + K * (Z - H * X)
        Eigen::Vector3f residual = v_obs - H * v_b;
        v_b = v_b + K * residual;

        // 更新协方带：P = (I - K * H) * P
        P = (Eigen::Matrix3f::Identity() - K * H) * P;
    }

    // 返回当前速度
    Eigen::Vector3f getVelocity() const {
        return v_b;
    }

private:
    Eigen::Vector3f v_b; // 估计的速度
    float dt;            // 时间段
    float g;             // 重力加速度
    Eigen::Matrix3f Q;   // 过程器出误协方带
    Eigen::Matrix3f R;   // 观测器出误协方带
    Eigen::Matrix3f P;   // 状态协方带

    // 四元数转为方向举矩
    Eigen::Matrix3f quaternionToRotationMatrix(const float* quat) {
        float w = quat[0], x = quat[1], y = quat[2], z = quat[3];
        Eigen::Matrix3f R;
        R << 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
             2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
             2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y);
        return R;
    }

    // 计算关节的雅可比矩
    Eigen::Matrix3f computeJacobian(const std::vector<float>& q, int leg) {
        // 源根基于结构，实现雅可比矩计算
        return Eigen::Matrix3f::Identity();
    }
};
