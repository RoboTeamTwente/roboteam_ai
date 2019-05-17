//
// Created by kjhertenberg on 16-5-19.
//

#ifndef ROBOTEAM_WORLD_CONSTANTSK_H
#define ROBOTEAM_WORLD_CONSTANTSK_H

namespace rtt {
// constant dimensions of the calculations
const int STATEINDEX = 6;
const int OBSERVATIONINDEX = 3;
// timerate
const float TIMEDIFF = 0.01;
// time after which objects disappear
const float DISAPPEARTIME = 0.5*100; //seconds

// amount of robots and balls per team that we keep track off
const int BOTCOUNT=16; //id 0-15
const int BALLCOUNT=1;
const int TOTALCOUNT=BOTCOUNT*2+BALLCOUNT;
//used for checking convergence of K matrix
const float KMARGIN = 0.000001;
const int MAXCOMPARISONS = 100;

// constant variance estimates
const float posVar = 0.5;
const float stateVar = 1;
const float randVar = 1;
const float posVar_us = 0.5;
const float stateVar_us = 1;
const float randVar_us = 1;
const float posVar_them = 0.5;
const float stateVar_them = 1;
const float randVar_them = 1;
const float posVar_ball = 0.5;
const float stateVar_ball = 1;
const float randVar_ball = 1;


const unsigned int INVALID_ID = - 1;

};

#endif //ROBOTEAM_WORLD_CONSTANTSK_H
