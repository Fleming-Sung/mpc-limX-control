

#include "stateEstimator.h"
#include "limxsdk/datatypes.h"

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

/** Scalar type. */
using scalar_t = double;
/** Dynamic-size vector type. */
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;


stateEstimator::stateEstimator(ocs2::PinocchioInterface pinocchioInterface, const ocs2::PinocchioEndEffectorKinematics& eeKinematics)
    : pinocchioInterface_(std::move(pinocchioInterface)), 
    eeKinematics_(eeKinematics.clone()), 
    tfListener_(tfBuffer_),
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

Eigen::Matrix<double, Eigen::Dynamic, 1> stateEstimator::update(const ros::Time& time, const ros::Duration& period)
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


  updateLinear(xHat_.segment<3>(0), xHat_.segment<3>(3));

  auto odom = getOdomMsg();
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base";
  publishMsgs(odom);

  return rbdState_;
}

void stateEstimator::updateJointStates(const vector_t& jointPos, const vector_t& jointVel) {
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