#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

// #define BOOST_MPL_LIMIT_LIST_SIZE 50
// #define PINOCCHIO_BOOST_MPL_LIMIT_CONTAINER_SIZE 50


#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <realtime_tools/realtime_publisher.h>

#include "limxsdk/datatypes.h"



using namespace ocs2;
template <typename T>
using feet_array_t = std::array<T, 4>;
using contact_flag_t = feet_array_t<bool>;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using quaternion_t = Eigen::Quaternion<scalar_t>;

// 机器人状态结构
struct RobotOdomState {
  double pos[3];
  double ori[3];
  double quat[4];
  double v_pos[3];
  double v_ori[3];
};
// 机器人关节状态 from limx dynamics
struct RobotState {
    uint64_t stamp;            // 时间戳，单位是纳秒，表示数据记录的时间
    std::vector<float> tau;    // 当前转矩，单位为牛米（Nm）
    std::vector<float> q;      // 当前关节角，单位为弧度
    std::vector<float> dq;     // 当前关节速度，单位为弧速（rad/s）

    RobotState(int motor_num = 3) : tau(motor_num, 0.0), q(motor_num, 0.0), dq(motor_num, 0.0) {}
};

// 双足机器人urdf信息
std::vector<std::string> jointNames{"abad_L_Joint", "hip_L_Joint", "knee_L_Joint", "abad_R_Joint", "hip_R_Joint", "knee_R_Joint"};
std::vector<std::string> contactNames6DoF{};
std::vector<std::string> contactNames3DoF{"contact_L_Link", "contact_R_Link"};
template <typename T>

T square(T a) {
  return a * a;
}
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q) {
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;

  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}

class stateEstimator {

 public:
  stateEstimator(ocs2::PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const ocs2::PinocchioEndEffectorKinematics& eeKinematics);

  vector_t update(const ros::Time& time, const ros::Duration& period);

  void loadSettings(const std::string& taskFile, bool verbose);

  void updateJointStates(const vector_t& jointPos, const vector_t& jointVel);
  void updateContact(contact_flag_t contactFlag) { contactFlag_ = contactFlag; }
  void updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal, const vector3_t& linearAccelLocal,
                         const matrix3_t& orientationCovariance, const matrix3_t& angularVelCovariance,
                         const matrix3_t& linearAccelCovariance);
  
  RobotOdomState robotOdomState_;

 protected:

  void callback(const nav_msgs::Odometry::ConstPtr& msg);
  void updateAngular(const vector3_t& zyx, const vector_t& angularVel);
  void updateLinear(const vector_t& pos, const vector_t& linearVel);
  void publishMsgs(const nav_msgs::Odometry& odom);


  nav_msgs::Odometry getOdomMsg();

  Eigen::Matrix<double, Eigen::Dynamic, 1> feetHeights_;
    
   // Config
  double footRadius_ = 0.02;
  double imuProcessNoisePosition_ = 0.02;
  double imuProcessNoiseVelocity_ = 0.02;
  double footProcessNoisePosition_ = 0.002;
  double footSensorNoisePosition_ = 0.005;
  double footSensorNoiseVelocity_ = 0.1;
  double footHeightSensorNoise_ = 0.01;   

  ocs2::PinocchioInterface pinocchioInterface_;
  ocs2::CentroidalModelInfoTpl<double> info_;
  std::unique_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematics_;
  Eigen::Matrix<double, Eigen::Dynamic, 1>  rbdState_;
  contact_flag_t contactFlag_{};
  vector3_t angularVelLocal_, linearAccelLocal_;
  matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;
  Eigen::Quaternion<scalar_t> quat_;
  vector3_t zyxOffset_ = vector3_t::Zero();

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odomPub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>> posePub_;
  ros::Time lastPub_;




 private:
  Eigen::Matrix<double, 12, 1> xHat_;   // baselink 位置，速度，双足的位置 
  Eigen::Matrix<double, 6, 1> ps_;      // 双足的位置
  Eigen::Matrix<double, 6, 1> vs_;      // 双足的速度
  Eigen::Matrix<double, 12, 12> a_;     // 状态转移矩阵
  Eigen::Matrix<double, 12, 12> q_;     // 过程噪声协方差矩阵
  Eigen::Matrix<double, 12, 12> p_;     // 状态协方差矩阵
  Eigen::Matrix<double, 14, 14> r_;     // 观测噪声协方差矩阵
  Eigen::Matrix<double, 12, 3> b_;      // 控制矩阵
  Eigen::Matrix<double, 14, 12> c_;     // 观测矩阵

    // Topic
  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
  // tf2_ros::Buffer tfBuffer_;
  // tf2_ros::TransformListener tfListener_;
  // tf2::Transform world2odom_;
  std::string frameOdom_, frameGuess_;
  bool topicUpdated_;
  const int SingleRigidBodyDynamics = 1; 
};




/** Scalar type. */
using scalar_t = double;
/** Dynamic-size vector type. */
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;


stateEstimator::stateEstimator(ocs2::PinocchioInterface pinocchioInterface,  CentroidalModelInfo info,const ocs2::PinocchioEndEffectorKinematics& eeKinematics)
    : pinocchioInterface_(std::move(pinocchioInterface)), 
      eeKinematics_(eeKinematics.clone()), 
      info_(std::move(info)),
    // tfListener_(tfBuffer_),
      rbdState_(vector_t ::Zero(2 * info_.generalizedCoordinatesNum))
{
  // 声明发布的话题
  ros::NodeHandle nh;
  odomPub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(nh, "odom", 10));

  posePub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>(nh, "pose", 10));

  // 定义状态方程和卡尔曼滤波的矩阵
  xHat_.setZero();
  ps_.setZero();
  vs_.setZero();
  a_.setZero();
  a_.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
  a_.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
  a_.block(6, 6, 6, 6) = Eigen::Matrix<double, 6, 6>::Identity(); //a是一个单位矩阵
  b_.setZero();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> c1(3, 6);
  c1 << Eigen::Matrix<double, 3, 3>::Identity(), Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> c2(3, 6);
  c2 << Eigen::Matrix<double, 3, 3>::Zero(), Eigen::Matrix<double, 3, 3>::Identity();
  c_.setZero();
  c_.block(0, 0, 3, 6) = c1;
  c_.block(3, 0, 3, 6) = c1;
  c_.block(0, 6, 6, 6) = -Eigen::Matrix<double, 6, 6>::Identity();
  c_.block(6, 0, 3, 6) = c2;
  c_.block(9, 0, 3, 6) = c2;
  c_(12, 8) = 1.0; 
  c_(13, 11) = 1.0;
  p_.setIdentity();
  p_ = 100. * p_;
  q_.setIdentity();
  r_.setIdentity();
  feetHeights_.setZero(4);
  eeKinematics_->setPinocchioInterface(pinocchioInterface_);


}

vector_t stateEstimator::update(const ros::Time& time, const ros::Duration& period)
{
  // (不是很理解）)
  double dt = period.toSec();
  a_.block(0, 3, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
  b_.block(0, 0, 3, 3) = 0.5 * dt * dt * Eigen::Matrix<double, 3, 3>::Identity();
  b_.block(3, 0, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
  q_.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
  q_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
  q_.block(6, 6, 6, 6) = dt * Eigen::Matrix<double, 12, 12>::Identity();

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  size_t actuatedDofNum = info_.actuatedDofNum;

  Eigen::Matrix<double, Eigen::Dynamic, 1> qPino(info_.generalizedCoordinatesNum);
  Eigen::Matrix<double, Eigen::Dynamic, 1> vPino(info_.generalizedCoordinatesNum);
  qPino.setZero();
  qPino.segment<3>(3) = rbdState_.head<3>();  // Only set orientation, let position in origin.
  qPino.tail(actuatedDofNum) = rbdState_.segment(6, actuatedDofNum);

  vPino.setZero();
  vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qPino.segment<3>(3),
      rbdState_.segment<3>(info_.generalizedCoordinatesNum));  // Only set angular velocity, let linear velocity be zero
  vPino.tail(actuatedDofNum) = rbdState_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);

  pinocchio::forwardKinematics(model, data, qPino, vPino);
  pinocchio::updateFramePlacements(model, data);
  // 实际落足点位置
  const auto eePos = eeKinematics_->getPosition(vector_t());
  const auto eeVel = eeKinematics_->getVelocity(vector_t(), vector_t());

  Eigen::Matrix<scalar_t, 12, 12> q = Eigen::Matrix<scalar_t, 12, 12>::Identity();
  q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imuProcessNoisePosition_;
  q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imuProcessNoiseVelocity_;
  q.block(6, 6, 6, 6) = q_.block(6, 6, 6, 6) * footProcessNoisePosition_;

  Eigen::Matrix<scalar_t, 14, 14> r = Eigen::Matrix<scalar_t, 14, 14>::Identity();
  r.block(0, 0, 6, 6) = r_.block(0, 0, 6, 6) * footSensorNoisePosition_;
  r.block(6, 6, 6, 6) = r_.block(6, 6, 6, 6) * footSensorNoiseVelocity_;
  r.block(12, 12, 2, 2) = r_.block(12, 12, 2, 2) * footHeightSensorNoise_;

  for (int i = 0; i < 2; i++) {
    int i1 = 3 * i;

    int qIndex = 6 + i1;
    int rIndex1 = i1;
    int rIndex2 = 6 + i1;
    int rIndex3 = 12 + i;
    // 这里还需要和limx的isContact融合
    bool isFootContact = contactFlag_[i];

    scalar_t high_suspect_number(100);
    q.block(qIndex, qIndex, 3, 3) = (isFootContact ? 1. : high_suspect_number) * q.block(qIndex, qIndex, 3, 3);
    r.block(rIndex1, rIndex1, 3, 3) = (isFootContact ? 1. : high_suspect_number) * r.block(rIndex1, rIndex1, 3, 3);
    r.block(rIndex2, rIndex2, 3, 3) = (isFootContact ? 1. : high_suspect_number) * r.block(rIndex2, rIndex2, 3, 3);
    r(rIndex3, rIndex3) = (isFootContact ? 1. : high_suspect_number) * r(rIndex3, rIndex3);

    ps_.segment(3 * i, 3) = -eePos[i];
    ps_.segment(3 * i, 3)[2] += footRadius_;
    vs_.segment(3 * i, 3) = -eeVel[i];
  }
  vector3_t g(0, 0, -9.81);
  vector3_t accel = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)).transpose() * linearAccelLocal_ + g;

  Eigen::Matrix<scalar_t, 14, 1> y;
  y << ps_, vs_, feetHeights_;
  xHat_ = a_ * xHat_ + b_ * accel;
  Eigen::Matrix<scalar_t, 12, 12> at = a_.transpose();
  Eigen::Matrix<scalar_t, 12, 12> pm = a_ * p_ * at + q;
  Eigen::Matrix<scalar_t, 12, 14> cT = c_.transpose();
  Eigen::Matrix<scalar_t, 14, 1> yModel = c_ * xHat_;
  Eigen::Matrix<scalar_t, 14, 1> ey = y - yModel;
  Eigen::Matrix<scalar_t, 14, 14> s = c_ * pm * cT + r;

  Eigen::Matrix<scalar_t, 14, 1> sEy = s.lu().solve(ey);
  xHat_ += pm * cT * sEy;

  Eigen::Matrix<scalar_t, 14, 12> sC = s.lu().solve(c_);
  p_ = (Eigen::Matrix<scalar_t, 12, 12>::Identity() - pm * cT * sC) * pm;

  Eigen::Matrix<scalar_t, 12, 12> pt = p_.transpose();
  p_ = (p_ + pt) / 2.0;

  if (p_.block(0, 0, 2, 2).determinant() > 0.000001) {
    p_.block(0, 2, 2, 10).setZero();
    p_.block(2, 0, 10, 2).setZero();
    p_.block(0, 0, 2, 2) /= 10.;
  }

  // std::cout<<"xHat_:"<<xHat_<<std::endl;

  updateLinear(xHat_.segment<3>(0), xHat_.segment<3>(3));

  auto odom = getOdomMsg();
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base";
  publishMsgs(odom);

  // 把xHat_的状态信息储存在robotOdomState_
  robotOdomState_.pos[0] = xHat_.segment<3>(0)(0);
  robotOdomState_.pos[1] = xHat_.segment<3>(0)(1);
  robotOdomState_.pos[2] = xHat_.segment<3>(0)(2);
  robotOdomState_.quat[0] = quat_.x();
  robotOdomState_.quat[1] = quat_.y();
  robotOdomState_.quat[2] = quat_.z();
  robotOdomState_.quat[3] = quat_.w();
  vector_t twist = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)).transpose() * xHat_.segment<3>(3);
  robotOdomState_.v_pos[0] = twist.x();
  robotOdomState_.v_pos[1] = twist.y();
  robotOdomState_.v_pos[2] = twist.z();
  robotOdomState_.v_ori[0] = angularVelLocal_.x();
  robotOdomState_.v_ori[1] = angularVelLocal_.y();
  robotOdomState_.v_ori[2] = angularVelLocal_.z();

  // std::cout<<"robotOdomState_:"<<robotOdomState_.pos[0]<<"  "<<robotOdomState_.pos[1]<<"  "<<robotOdomState_.pos[2]<<"  "<<std::endl;

  return rbdState_;
}

void stateEstimator::updateJointStates(const vector_t& jointPos, const vector_t& jointVel) {
  // std::cout<<"info_.generalizedCoordinatesNum:"<<info_.generalizedCoordinatesNum<<std::endl;
  rbdState_.segment(6, info_.actuatedDofNum) = jointPos;
  rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = jointVel;
}

void stateEstimator::updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal,
                                  const vector3_t& linearAccelLocal, const matrix3_t& orientationCovariance,
                                  const matrix3_t& angularVelCovariance, const matrix3_t& linearAccelCovariance) {
  quat_ = quat;
  angularVelLocal_ = angularVelLocal;
  linearAccelLocal_ = linearAccelLocal;
  orientationCovariance_ = orientationCovariance;
  angularVelCovariance_ = angularVelCovariance;
  linearAccelCovariance_ = linearAccelCovariance;

  vector3_t zyx = quatToZyx(quat) - zyxOffset_;
  vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
      zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat), angularVelLocal));
  updateAngular(zyx, angularVelGlobal);
}

void stateEstimator::updateAngular(const vector3_t& zyx, const vector_t& angularVel) {
  rbdState_.segment<3>(0) = zyx;
  rbdState_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;
}

void stateEstimator::updateLinear(const vector_t& pos, const vector_t& linearVel) {
  rbdState_.segment<3>(3) = pos;
  rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;
}

nav_msgs::Odometry stateEstimator::getOdomMsg() {
  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = xHat_.segment<3>(0)(0);
  odom.pose.pose.position.y = xHat_.segment<3>(0)(1);
  odom.pose.pose.position.z = xHat_.segment<3>(0)(2);
  odom.pose.pose.orientation.x = quat_.x();
  odom.pose.pose.orientation.y = quat_.y();
  odom.pose.pose.orientation.z = quat_.z();
  odom.pose.pose.orientation.w = quat_.w();
  odom.pose.pose.orientation.x = quat_.x();
  for (int i = 0; i < 1; ++i) {
    for (int j = 0; j < 1; ++j) {
      odom.pose.covariance[i * 6 + j] = p_(i, j);
      odom.pose.covariance[6 * (3 + i) + (3 + j)] = orientationCovariance_(i * 3 + j);
    }
  }
  //  The twist in this message should be specified in the coordinate frame given by the child_frame_id: "base"
  vector_t twist = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)).transpose() * xHat_.segment<3>(3);
  odom.twist.twist.linear.x = twist.x();
  odom.twist.twist.linear.y = twist.y();
  odom.twist.twist.linear.z = twist.z();
  odom.twist.twist.angular.x = angularVelLocal_.x();
  odom.twist.twist.angular.y = angularVelLocal_.y();
  odom.twist.twist.angular.z = angularVelLocal_.z();
  for (int i = 0; i < 1; ++i) {
    for (int j = 0; j < 1; ++j) {
      odom.twist.covariance[i * 6 + j] = p_.block<3, 3>(3, 3)(i, j);
      odom.twist.covariance[6 * (3 + i) + (3 + j)] = angularVelCovariance_(i * 3 + j);
    }
  }
  return odom;
}

void stateEstimator::publishMsgs(const nav_msgs::Odometry& odom) {
  ros::Time time = odom.header.stamp;
  scalar_t publishRate = 200;
  if (lastPub_ + ros::Duration(1. / publishRate) < time) {
    lastPub_ = time;
    if (odomPub_->trylock()) {
      odomPub_->msg_ = odom;
      odomPub_->unlockAndPublish();
    }
    if (posePub_->trylock()) {
      posePub_->msg_.header = odom.header;
      posePub_->msg_.pose = odom.pose;
      posePub_->unlockAndPublish();
    }
  }
}






#endif