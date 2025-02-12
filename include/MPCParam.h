#ifndef _MPCParam_H_
#define _MPCParam_H_

#include "limxsdk/datatypes.h"
#include <cmath>

// 定义Vec3类型为Eigen的三维向量
typedef Eigen::Matrix<double, 3, 1> Vec3;

struct kinematicValues {
// Abad offset from base frame values
double abad_offset_x = 0.05556;
double abad_offset_y = 0.105;
double abad_offset_z = -0.2602;

// Hip offset from abad frame values
double hip_offset_x = -0.077;
double hip_offset_y = 0.02050;
double hip_offset_z = 0.0;

// Knee offset from abad frame values
double knee_offset_x = -0.1500;
double knee_offset_y = -0.02050;
double knee_offset_z = -0.25981;

// Foot offset from abad frame values
double foot_offset_x = 0.145;
double foot_offset_y = 0.0;
double foot_offset_z = -0.2598;

// Contact offset from abad frame values
double contact_offset_x = 0.0;
double contact_offset_y = 0.0;
double contact_offset_z = -0.032;
};

class MPCParam{
public:
    MPCParam();

    float dt = 0.001;
    int milliseconds_per_step = static_cast<int>(1/dt);
    int mpcStep = 5;
    float dtMPC = dt * mpcStep;
    float swing_time = 0.5;
    float stance_time = 0.5;

    float givenErrorRate = 0.1;

    kinematicValues KinematicValues;
    Vec3 static_foot_offset;    

    bool errorTest(std::vector<float> targetPos, std::vector<float> nowPos);  
              
    
};

MPCParam::MPCParam(){
    // 默认状态下，baselink到地面接触点的偏移量
    static_foot_offset << KinematicValues.abad_offset_x + KinematicValues.hip_offset_x + KinematicValues.knee_offset_x + KinematicValues.foot_offset_x + KinematicValues.contact_offset_x,
                          KinematicValues.abad_offset_y + KinematicValues.hip_offset_y + KinematicValues.knee_offset_y + KinematicValues.foot_offset_y + KinematicValues.contact_offset_y,
                          KinematicValues.abad_offset_z + KinematicValues.hip_offset_z + KinematicValues.knee_offset_z + KinematicValues.foot_offset_z + KinematicValues.contact_offset_z;
}

bool MPCParam::errorTest(std::vector<float> targetPos, std::vector<float> nowPos)
{
    bool errorFlag = true;
    for(int i=0; i<6; i++){
        if (abs(targetPos[i]-nowPos[i]) >= givenErrorRate){errorFlag=false;}
    }
    return errorFlag;
}

#endif 
