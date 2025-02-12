#include "limxsdk/datatypes.h"
#include "MPCParam.h"
#include "state_estimator_fake.h"
#include <Eigen/Dense> // 引入Eigen库

// 定义Vec3类型为Eigen的三维向量
typedef Eigen::Matrix<double, 3, 1> Vec3;

class MPC {
public:
    MPC();
    void run(limxsdk::RobotState state, limxsdk::ImuData imu, limxsdk::RobotCmd& cmd, int iter);
    MPCParam param;
    StateEstimatorFake estimates;

    Vec3 desieredV_pos = {1.0, 0.0, 0.0};
    Vec3 desieredV_ori = {0.0, 0.0, 0.0};

private:
    RobotOdomState odmo_state; // 状态估计结果，包括baselin的位置、朝向、速度、角速度
    int left_leg_state;
    int right_leg_state;       //每条腿的宏观状态，0表示支撑，1表示摆动
    double phase;              // 当前步态周期中的相位（时间表示，单位为s）
    double remainSwingTime;    // 剩余摆动时间

    void calculateGait(int iter);
    void computeFootPlacement(limxsdk::RobotState state, int leg, Vec3& finalPosition); // 计算摆动腿落脚点
    void computeSwingFootDesiredPosition(int leg, double swingPhase, Vec3& desiredPosition, Vec3& desiredVelocity); // 计算摆动腿期望位置
    void computeJointPositions(Vec3 desiredFootPosition, int leg, limxsdk::RobotCmd& cmd); // 通过逆运动学计算关节期望位置
};

MPC::MPC() {
    odmo_state = estimates.get_state();
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

// 根据期望速度和当前位置计算摆动腿下一次落地的落足点位置
void MPC::computeFootPlacement(limxsdk::RobotState state, int leg, Vec3& finalPosition) {
    Vec3 currentPosition;
    Vec3 currentVelocity;
    currentPosition << odmo_state.pos[0], odmo_state.pos[1], odmo_state.pos[2];
    currentVelocity << odmo_state.v_pos[0], odmo_state.v_pos[1], odmo_state.v_pos[2];

    //如果保持当前速度运行，摆动腿下一次落地时期望的baselink位置
    Vec3 predictedPosition = currentPosition + currentVelocity * remainSwingTime; 
    
    // 当前摆动腿落地后变为支撑腿后，使得baselink继续运动，直到支撑相的一半时
    double p_rel_max = 0.3;                                              // 最大偏移距离
    // 保持当前运动速度产生的偏移及其与期望速度的修正
    double pfx_rel = currentVelocity[0] * 0.5 * param.stance_time; // TODO：未添加期望速度修正  
    double pfy_rel = currentVelocity[1] * 0.5 * param.stance_time;
    // 最大偏移距离的修正
    pfx_rel = fmin(fmax(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fmin(fmax(pfy_rel, -p_rel_max), p_rel_max);

    predictedPosition[0] += pfx_rel; // 到达下一个支撑相中间时，baselink的期望位置
    predictedPosition[1] += pfy_rel;
    predictedPosition[2] = 0.5;

    // 假设下一个支撑相中间时，摆动腿到达static offset姿态；
    // 根据此时的预测baselink位置(predictedPosition)反推出当前摆动腿的落足点位置(世界坐标系下的绝对位置)
    finalPosition = predictedPosition + param.static_foot_offset;
}

void MPC::computeSwingFootDesiredPosition(int leg, double swingPhase, Vec3& desiredPosition, Vec3& desiredVelocity) {
    Vec3 startPosition(0.0, 0.0, 0.0);
    Vec3 endPosition(0.3, 0.0, 0.0);
    double swingHeight = 0.1;

    double t = swingPhase;
    desiredPosition[0] = (1 - t) * startPosition[0] + t * endPosition[0];
    desiredPosition[1] = (1 - t) * startPosition[1] + t * endPosition[1];
    desiredPosition[2] = swingHeight * 4 * t * (1 - t);

    desiredVelocity[0] = (endPosition[0] - startPosition[0]);
    desiredVelocity[1] = (endPosition[1] - startPosition[1]);
    desiredVelocity[2] = swingHeight * 4 * (1 - 2 * t);
}

void MPC::computeJointPositions(Vec3 desiredFootPosition, int leg, limxsdk::RobotCmd& cmd) {
    double upperLegLength = param.upper_leg_length;
    double lowerLegLength = param.lower_leg_length;

    double x = desiredFootPosition[0];
    double y = desiredFootPosition[1];
    double z = desiredFootPosition[2];

    double hipYawAngle = atan2(y, x);
    double planarDistance = sqrt(x * x + y * y);
    double hipPitchAngle = atan2(z, planarDistance);
    double legLength = sqrt(planarDistance * planarDistance + z * z);
    double kneeAngle = acos((upperLegLength * upperLegLength + lowerLegLength * lowerLegLength - legLength * legLength) / (2 * upperLegLength * lowerLegLength));

    cmd.q[leg * 3 + 0] = hipYawAngle;
    cmd.q[leg * 3 + 1] = hipPitchAngle;
    cmd.q[leg * 3 + 2] = kneeAngle;
}

void MPC::run(limxsdk::RobotState state, limxsdk::ImuData imu, limxsdk::RobotCmd& cmd, int iter) {
    int left_leg_state = 0;
    int right_leg_state = 0;

    double swingTime = param.swing_time;
    calculateGait(iter, swingTime, left_leg_state, right_leg_state);

    Vec3 leftFootPlacement, rightFootPlacement;

    if (left_leg_state == 1) {
        computeFootPlacement(state, 0, leftFootPlacement);
    }
    if (right_leg_state == 1) {
        computeFootPlacement(state, 1, rightFootPlacement);
    }

    double swingPhase = fmod(iter * param.control_dt, param.swing_time) / param.swing_time;
    Vec3 leftDesiredPosition, leftDesiredVelocity;
    Vec3 rightDesiredPosition, rightDesiredVelocity;

    if (left_leg_state == 1) {
        computeSwingFootDesiredPosition(0, swingPhase, leftDesiredPosition, leftDesiredVelocity);
        computeJointPositions(leftDesiredPosition, 0, cmd);
    }
    if (right_leg_state == 1) {
        computeSwingFootDesiredPosition(1, swingPhase, rightDesiredPosition, rightDesiredVelocity);
        computeJointPositions(rightDesiredPosition, 1, cmd);
    }
}