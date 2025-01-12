#ifndef _MPCParam_H_
#define _MPCParam_H_

#include "limxsdk/datatypes.h"
#include <cmath>

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

    bool errorTest(std::vector<float> targetPos, std::vector<float> nowPos);  
              
    
};

MPCParam::MPCParam(){
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
