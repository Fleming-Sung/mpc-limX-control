/**
 * @file pf_groupJoints_move.cpp
 * @brief Implementation file for controlling movement of multiple joints simultaneously.
 * @version 1.0
 * @date 2024-3-6
 *
 * © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.
 *
 */

#include "pf_controller_base.h" // Include header file for PFControllerBase class
#include "stateEstimator.h" // Include for stateEstimator class
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ros/ros.h>

// Class for controlling movement of multiple joints simultaneously inheriting from PFControllerBase
class PFGroupJointMove : public PFControllerBase
{
public:
  /**
   * @brief Initialize the controller.
   */
  void init()
  {
    // Subscribing to diagnostic values for calibration state
    pf_->subscribeDiagnosticValue([&](const limxsdk::DiagnosticValueConstPtr& msg) {
      // Check if the diagnostic message pertains to calibration
      if (msg->name == "calibration") {
        if (msg->code != 0){
          abort();
        }
      }
    });
    
    // Set default values for gains, target positions, velocities, and torques
    kp.resize(pf_->getMotorNumber(), 60.0);
    kd.resize(pf_->getMotorNumber(), 3.0);
    targetPos.resize(pf_->getMotorNumber(), 0.0);
    targetVel.resize(pf_->getMotorNumber(), 0.0);
    targetTorque.resize(pf_->getMotorNumber(), 0.0);
    init_pos_.resize(pf_->getMotorNumber(), 0.0);
    robotstate_on_ = false; // Initialize robot state flag

    // state estimator init
    setupStateEstimate();
    std::cout << "success init...\n";
    
  }

  /**
   * @brief Start the control loop for controlling multiple joints simultaneously.
   */
  void starting()
  {
    std::cout << "Waiting to receive data...\n";

    while (true)
    {
      if (robotstate_on_)
      {
        auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);
        std::vector<float> jointPos{6, 0.0}; // Vector to store desired joint positions
        double r = 0.0;                       // Variable to calculate interpolation ratio

        // If it's the first iteration, initialize the joint positions
        if (is_first_enter_)
        {
          init_pos_ = robot_state_.q;
          is_first_enter_ = false;
          std::cout << "Received\n";
        }

        // Calculate the interpolation ratio
        r = std::min(std::max(double(running_iter_) / 2000.0, 0.0), 1.0);

        // Calculate the desired joint positions using linear interpolation
        for (size_t i = 0; i < pf_->getMotorNumber(); ++i)
        {
          jointPos[i] = (1 - r) * init_pos_[i] + r * targetPos[i];
        }

        // Control the joints using PID controllers
        groupJointController(kp, kd, jointPos, targetVel, targetTorque);

        std::this_thread::sleep_until(time_point); // Sleep until the next iteration

        if (!is_first_enter_)
        {
          running_iter_++; // Increment the iteration count
        }
        robotstate_on_ = false; // Reset the flag for receiving robot state data

        // update estimated state
        updateStateEstimation(ros::Time::now(), ros::Duration(0.001), robot_state_, imu_data_); // Update the state estimation
      }
      else
      {
        usleep(1); // Sleep for a short duration if robot state data is not received
      }
    }
  }

  // Function to get the robot state
  void updateStateEstimation(const ros::Time& time, const ros::Duration& period, limxsdk::RobotState robot_state, limxsdk::ImuData imu_data)
  {
      // state estimate
    vector_t jointPos(6), jointVel(6);
    contact_flag_t contacts;
    Eigen::Quaternion<scalar_t> quat;
    contact_flag_t contactFlag = {true, true};
    vector3_t angularVel, linearAccel;
    matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;


    // joint 
    for (int i = 0; i < 6; ++i) {
      jointPos(i) = robot_state_.q[i];
      jointVel(i) = robot_state_.dq[i];
    }
    // quat
    for (int i = 0; i < 4; ++i) {
      quat.coeffs()(i) = imu_data_.quat[i];
    }
    // acceleration
    for (int i = 0; i < 3; ++i) {
      linearAccel(i) = imu_data_.acc[i];
    }
    // angular velocity
    for (int i = 0; i < 3; ++i) {
      angularVel(i) = imu_data_.gyro[i];
    }

    stateEstimate_->updateJointStates(jointPos, jointVel);
    stateEstimate_->updateContact(contactFlag);
    stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
    measuredRbdState_ = stateEstimate_->update(time, period);

  }

  void setupStateEstimate() {

    // setupModel
    // PinocchioInterface
    pinocchioInterfacePtr_ =
        std::make_unique<PinocchioInterface>(centroidal_model::createPinocchioInterface(urdfFile_, jointNames));

    // CentroidalModelInfo
    centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
        *pinocchioInterfacePtr_, CentroidalModelType::SingleRigidBodyDynamics,
        defaultJointState_, contactNames3DoF,
        contactNames6DoF);
    
    CentroidalModelPinocchioMapping pinocchioMapping(centroidalModelInfo_);
    eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(*pinocchioInterfacePtr_, pinocchioMapping, 
                                                                        contactNames3DoF);

    stateEstimate_ = std::make_shared<stateEstimator>(*pinocchioInterfacePtr_,
                                                      centroidalModelInfo_, 
                                                      *eeKinematicsPtr_);
  }

private:
  std::vector<float> kp, kd, targetPos, targetVel, targetTorque; // Gains and targets
  std::vector<float> init_pos_;                                  // Initial joint positions
  bool is_first_enter_{true};                                    // Flag for first iteration
  int running_iter_{1};                                          // Iteration count

  // stateEstimator 变量
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  CentroidalModelInfo centroidalModelInfo_;
  vector_t measuredRbdState_;
  std::shared_ptr<stateEstimator> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

  std::string urdfFile_ = "/home/george/code/limx_ws/src/robot-description/pointfoot/PF_TRON1A/urdf/robot.urdf";
  vector_t defaultJointState_ = vector_t::Zero(6);
};

/**
 * @brief Main function.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Integer indicating the exit status.
 */
int main(int argc, char *argv[])
{
  ros::init(argc, argv,"pf_groupJoints_move_test");

  limxsdk::PointFoot *pf = limxsdk::PointFoot::getInstance(); // Obtain instance of PointFoot class

  std::string robot_ip = "127.0.0.1"; // Default robot IP address
  if (argc > 1)
  {
    robot_ip = argv[1]; // Use command-line argument as robot IP address if provided
  }

  // Initialize the robot
  if (!pf->init(robot_ip))
  {
    exit(1); // Exit program if initialization fails
  }

  PFGroupJointMove ctrl; // Create an instance of PFGroupJointMove controller
  ctrl.init();           // Initialize the controller
  ctrl.starting();       // Start the control loop

  // Infinite loop to keep the program running
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep for 100 milliseconds
  }

#ifdef WIN32
  timeEndPeriod(1);
#endif

  return 0; // Return 0 to indicate successful execution
}
