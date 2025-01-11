#include "MPCController.h"
#include "limxsdk/datatypes.h"


MPC::MPC(){
}

void MPC::run(limxsdk::RobotState state, limxsdk::ImuData imu, limxsdk::RobotCmd& cmd, int iter) 
{
    float tau;
    tau = 20.0;
    if (iter%1000 >= 500){
        tau = -tau;
    }
    cmd.tau[1] = tau;
    cmd.tau[4] = tau;
}