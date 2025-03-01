/**
 * @file moc_control.cpp
 * @brief Implementation file for MPC control of LimX P1 robot walking with fixed gait.
 * @version 1.0
 * @date 2024-12-9
 *
 * © [2024] Fleming@HIT, All rights reserved.
 * E-mail: fleming.ming.sung@gmail.com
 *
 */

 #include "pf_controller_base.h" // Include header file for PFControllerBase class
 #include "MPCController.h"
 #include "MPCParam.h"
 #include "ros/ros.h"
 
 // Class for controlling movement of multiple joints simultaneously inheriting from PFControllerBase
 class MPCWalking : public PFControllerBase
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
     kp = {60, 60, 60, 60, 60, 60};
     kd = {3, 3, 3, 3, 3, 3};
     targetPos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
     targetVel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
     targetTorque = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
     robotstate_on_ = false; // Initialize robot state flag
   }
 
   /**
    * @brief Control the robot joints move to the zero point.
    */
   void start()
   {
     bool reachFlag = false;
 
     while (!reachFlag)
     { 
      //  pf_ -> subscribeRobotState();
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
           std::cout << "Moving to zero point...\n"<< std::endl;
         }
 
         // Calculate the interpolation ratio
         r = std::min(std::max(double(running_iter_) / 2000.0, 0.0), 1.0);
 
         // Calculate the desired joint positions using linear interpolation
         for (int i = 0; i < getNumofJoint(); ++i)
         {
           jointPos[i] = (1 - r) * init_pos_[i] + r * targetPos[i];
         }
 
         // Control the joints using PID controllers
         groupJointController(kp, kd, jointPos, targetVel, targetTorque);
         std::cout << "now q:\t" << robot_state_.q[0] << robot_state_.q[1] << robot_state_.q[2] << robot_state_.q[3] << robot_state_.q[4] << robot_state_.q[5] << std::endl;
 
         // Check if reached the zero point with given error.
         reachFlag = mpc.param.errorTest(targetPos, robot_state_.q);
         
         std::this_thread::sleep_until(time_point); // Sleep until the next iteration
         // std::cout << reachFlag << std::endl;
 
         if (!is_first_enter_)
         {
           running_iter_++; // Increment the iteration count
         }
         robotstate_on_ = false; // Reset the flag for receiving robot state data
       }
       else
       {
         //  std::this_thread::sleep_for(std::chrono::seconds(1)); // Sleep for a short duration if robot state data is not received
         usleep(1);
         std::cout << "Waiting for robot state data..." << std::endl;
       }
     }
     std::cout << "Moved to zero point within accuracy of:\t" << mpc.param.givenErrorRate << std::endl;
   }
 
 
   /**
    * @brief Robot control runtime fuction.
    */
 void run()
   {
    //  for (int i=0;i<6;i++){
    //    robot_cmd_.mode[i] = 0; // mode 0 for torque control 
    //  }
     
     // if TODO, begin MPC walking.
     auto init_time_point = std::chrono::steady_clock::now();
     while (true)
     {
       if (robotstate_on_)
       {
         std::cout << "MPC is running..." << std::endl;
 
         auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(mpc.param.milliseconds_per_step); // 将控制频率设定为mpc.param.milliseconds_per_step
 
         // begin MPC compute...
         mpc.run(robot_state_, imu_data_, robot_cmd_, running_iter_); // compute dest torque in robot_cmd.
 
         // std::cout << robot_state_ << std::endl;
 
         // publish control cmd to robot.
         // groupJointController(robot_cmd_.Kp, robot_cmd_.Kd, robot_cmd_.q, robot_cmd_.dq, robot_cmd_.tau);
         pf_->publishRobotCmd(robot_cmd_);
 
         std::this_thread::sleep_until(time_point); // Sleep until the next iteration
 
         if (!is_first_enter_)
         {
           running_iter_++; // Increment the iteration count
         }
         robotstate_on_ = false; // Reset the flag for receiving robot state data
       }
       else
       {
         //  std::this_thread::sleep_for(std::chrono::seconds(1)); // Sleep for a short duration if robot state data is not received
         usleep(1);
         init_time_point += std::chrono::seconds(1);
         std::cout << "Waiting for robot state data..." << std::endl;
       }
     }
   }
 
 private:
   std::vector<float> kp{6, 0.0}, kd{6, 0.0}, targetPos{6, 0.0}, targetVel{6, 0.0}, targetTorque{6, 0.0}; // Gains and targets
   std::vector<float> init_pos_{6, 0.0};                                                                   // Initial joint positions
   bool is_first_enter_{true};                                                                              // Flag for first iteration
   int running_iter_{1};    
   MPC mpc;
 };
 
 /**
  * @brief Main function.
  * @param argc Number of command-line arguments.
  * @param argv Array of command-line arguments.
  * @return Integer indicating the exit status.
  */
 int main(int argc, char *argv[])
 {
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
 
   // 初始化创建虚拟状态观测器节点（必须在MPCWalking类实例化之前进行）
   ros::init(argc, argv, "state_estimator_fake_node");
   ros::NodeHandle nh;
 
   MPCWalking controller; // Create an instance of PFGroupJointMove controller
   controller.init();           // Initialize the controller
   controller.start();       //  move to zero point
   controller.run();
 
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