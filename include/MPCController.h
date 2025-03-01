#include "limxsdk/datatypes.h"
#include "MPCParam.h"
#include "state_estimator_fake.h"
#include "mpcQP.h"
#include "pinocchio_kinematics.h"
#include <Eigen/Dense> // 引入Eigen库
#include <vector>

class MPC {
public:
    MPC();
    void run(limxsdk::RobotState state, limxsdk::ImuData imu, limxsdk::RobotCmd& cmd, int iter);
    MPCParam param;
    StateEstimatorFake estimates;

    Eigen::Vector3d desieredV_pos = Eigen::Vector3d(1.0, 0.0, 0.0);
    Eigen::Vector3d desieredV_ori = Eigen::Vector3d(0.0, 0.0, 0.0);

private:
    RobotOdomState odom_state; // 状态估计结果，包括baselin的位置、朝向、速度、角速度
    PinocchioKinematics kinematics; // 机器人运动学模型
    int left_leg_state;
    int right_leg_state;       //每条腿的宏观状态，0表示支撑，1表示摆动
    double phase;              // 当前步态周期中的相位（时间表示，单位为s）
    double remainSwingTime;    // 剩余摆动时间
    Eigen::Vector3d finalPosition; // 摆动腿下一次落地的落足点位置
    Eigen::Vector3d currentPosition;      // 当前baselink位置
    Eigen::Vector3d currentVelocity;       // 当前baselink速度
    Eigen::Vector3d currentOrientation;    // 当前baselink朝向
    Eigen::Vector3d currentAngularVelocity; // 当前baselink角速度
    Eigen::Vector4d currentQuat; // 当前baselink四元数

    void update_odom_state(); // 更新状态估计结果
    void calculateGait(int iter);
    void computeFootPlacement(Eigen::Vector3d& finalPosition); // 计算摆动腿落脚点
    void computeSwingFootDesiredPosition(limxsdk::RobotState& state, limxsdk::RobotCmd& cmd); // 计算摆动腿期望位置
    // 计算支撑腿地面反力
    void computeSupportFootForce(limxsdk::RobotState& state);
};

MPC::MPC() {
    odom_state = estimates.get_state(); // 初始化时会先进行一次状态估计
}

void MPC::update_odom_state() {
    odom_state = estimates.get_state();
    // 当前baselink的位置
    currentPosition << odom_state.pos[0], odom_state.pos[1], odom_state.pos[2];
    // 当前baselink的速度
    currentVelocity << odom_state.v_pos[0], odom_state.v_pos[1], odom_state.v_pos[2];
    // 当前baselink的朝向（欧拉角）
    currentOrientation << odom_state.ori[0], odom_state.ori[1], odom_state.ori[2];
    // 当前baselink的角速度
    currentAngularVelocity << odom_state.v_ori[0], odom_state.v_ori[1], odom_state.v_ori[2];
    // 当前baselink的四元数
    currentQuat << odom_state.quat[0], odom_state.quat[1], odom_state.quat[2], odom_state.quat[3];

}

// 按固定迈步周期计算当前两条腿分别是支撑状态还是摆动状态，1表示摆动，0表示支撑
void MPC::calculateGait(int iter) {
    double currentTime = iter * param.dt; // 控制周期时间间隔
    double cycleTime = param.swing_time + param.stance_time; // 完整的步态周期时间
    phase = fmod(currentTime, cycleTime); // 当前步态周期中的相位

    if (phase < param.swing_time) {
        left_leg_state = 1; // 左腿摆动，右腿支撑
        right_leg_state = 0;
        remainSwingTime = param.swing_time - phase;
    } else {
        left_leg_state = 0; // 右腿摆动，左腿支撑
        right_leg_state = 1;
        remainSwingTime = cycleTime - phase;
    }
}

// 根据当前速度和当前位置计算摆动腿下一次落地的落足点位置
// void MPC::computeFootPlacement(Eigen::Vector3d& finalPosition) {

//     //如果保持当前速度运行，摆动腿下一次落地时期望的baselink位置
//     Eigen::Vector3d predictedPosition = currentPosition + currentVelocity * remainSwingTime; 
    
//     // 当前摆动腿落地后变为支撑腿后，使得baselink继续运动，直到支撑相的一半时
//     double p_rel_max = 0.3;                                              // 最大偏移距离
//     // 保持当前运动速度产生的偏移及其与期望速度的修正
//     double pfx_rel = currentVelocity[0] * 0.5 * param.stance_time; // TODO：未添加期望速度修正  
//     double pfy_rel = currentVelocity[1] * 0.5 * param.stance_time;
//     // 最大偏移距离的修正
//     pfx_rel = fmin(fmax(pfx_rel, -p_rel_max), p_rel_max);
//     pfy_rel = fmin(fmax(pfy_rel, -p_rel_max), p_rel_max);

//     predictedPosition[0] += pfx_rel; // 到达下一个支撑相中间时，baselink的期望位置
//     predictedPosition[1] += pfy_rel;
//     predictedPosition[2] = 0.5;

//     // 假设下一个支撑相中间时，摆动腿到达static offset姿态；
//     // 根据此时的预测baselink位置(predictedPosition)反推出当前摆动腿的落足点位置(世界坐标系下的绝对位置)
//     if (left_leg_state == 1) {
//         finalPosition = predictedPosition + param.static_foot_offset_right;
//     } else {
//         finalPosition = predictedPosition + param.static_foot_offset_left;
//     }
// }

// 根据期望速度计算摆动腿下一次落地的落足点位置
void MPC::computeFootPlacement(Eigen::Vector3d& finalPosition) {
    // 计算摆动腿下一次落地的落足点位置
    Eigen::Vector3d predictedPosition = currentPosition + desieredV_pos * remainSwingTime;

    // 当前摆动腿落地后变为支撑腿后，使得baselink继续运动，直到支撑相的一半时
    double p_rel_max = 0.3;                                              // 最大偏移距离
    // 保持当前运动速度产生的偏移及其与期望速度的修正
    double pfx_rel = desieredV_pos[0] * 0.5 * param.stance_time;  
    double pfy_rel = desieredV_pos[1] * 0.5 * param.stance_time;
    // 最大偏移距离的修正
    pfx_rel = fmin(fmax(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fmin(fmax(pfy_rel, -p_rel_max), p_rel_max);

    predictedPosition[0] += pfx_rel; // 到达下一个支撑相中间时，baselink的期望位置
    predictedPosition[1] += pfy_rel;
    predictedPosition[2] = 0;

    // 假设下一个支撑相中间时，摆动腿到达static offset姿态；
    // 根据此时的预测baselink位置(predictedPosition)反推出当前摆动腿的落足点位置(世界坐标系下的绝对位置)
    if (left_leg_state == 1) {   // 左腿摆动     
        finalPosition[0] = predictedPosition[0] + param.static_foot_offset_left[0];
        finalPosition[1] = predictedPosition[1] + param.static_foot_offset_left[1];
    } else {                    // 右腿摆动
        finalPosition[0] = predictedPosition[0] + param.static_foot_offset_right[0];
        finalPosition[1] = predictedPosition[1] + param.static_foot_offset_right[1];        
    }
}

void MPC::computeSwingFootDesiredPosition(limxsdk::RobotState& state, limxsdk::RobotCmd& cmd) {
    // 设置 base_link 的位姿
    Eigen::Vector3d base_position(odom_state.pos[0], odom_state.pos[1], odom_state.pos[2]);
    Eigen::Quaterniond base_orientation(odom_state.quat[0], odom_state.quat[1], odom_state.quat[2], odom_state.quat[3]); // Identity quaternion
    kinematics.setBaseLinkPose(base_position, base_orientation);

    // 从RobotState中获取当前关节角度
    std::vector<double> q(state.q.begin(), state.q.end()); // 匹配数据类型
    Eigen::VectorXd jointPositions = Eigen::Map<Eigen::VectorXd>(q.data(), q.size());
    // 基于当前关节角度计算正运动学
    kinematics.forwardKinematics(jointPositions);

    // 获取当前摆动腿的末端位置
    Eigen::Vector3d footPosition;
    if (left_leg_state == 1) {
        footPosition = kinematics.getLinkPosition("contact_L_Link");
    } else {
        footPosition = kinematics.getLinkPosition("contact_R_Link");
    }

    // 根据剩余摆动时间线性插值计算下一时间步摆动腿末端位置
    Eigen::Vector3d nextFootPosition = 
    footPosition + (finalPosition - footPosition) * (param.swing_time - remainSwingTime) / param.swing_time;
    // 正弦波拟合摆动腿高度
    nextFootPosition[2] = param.gait_height * sin(M_PI * (param.swing_time - remainSwingTime) / param.swing_time);

    // 基于逆运动学计算摆动腿的关节位置
    Eigen::VectorXd nextJointPositions;
    if (left_leg_state == 1) {
        nextJointPositions = kinematics.inverseKinematics("contact_L_Link", nextFootPosition, jointPositions);
        // 将计算得到的关节位置赋值给RobotCmd
        for (int i = 0; i < 3; i++) {
            cmd.q[i] = nextJointPositions[i];
        }
    } else {
        nextJointPositions = kinematics.inverseKinematics("contact_R_Link", nextFootPosition, jointPositions);
        // 将计算得到的关节位置赋值给RobotCmd
        for (int i = 3; i < cmd.q.size(); i++) {
            cmd.q[i] = nextJointPositions[i];
        }
    }
}

// 计算支撑腿地面反力
void MPC::computeSupportFootForce(limxsdk::RobotState& state) {
    // 计算支撑腿地面反力
}


void MPC::run(limxsdk::RobotState state, limxsdk::ImuData imu, limxsdk::RobotCmd& cmd, int iter) {

    // 更新状态估计结果
    update_odom_state();

    // 计算当前步态周期中的两条腿的状态
    calculateGait(iter);

    // 计算摆动腿下一次落地的落足点位置
    computeFootPlacement(finalPosition);

    // 计算摆动腿期望位置并更新RobotCmd
    computeSwingFootDesiredPosition(state, cmd);
}