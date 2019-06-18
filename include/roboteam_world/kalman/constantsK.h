//
// Created by kjhertenberg on 16-5-19.
//

#ifndef ROBOTEAM_WORLD_CONSTANTSK_H
#define ROBOTEAM_WORLD_CONSTANTSK_H

namespace rtt {
// constant dimensions of the calculations
const int STATEINDEX = 4;
const int OBSERVATIONINDEX = 2;
// timerate
const float TIMEDIFF = 0.01;
// time after which objects disappear
const float DISAPPEARTIME = 0.1/TIMEDIFF; //seconds/TIMEDIFF

// amount of robots and balls per team that we keep track off
const int BOTCOUNT=16; //id 0-15
const int BALLCOUNT=1;
const int TOTALCOUNT=BOTCOUNT*2+BALLCOUNT;
//used for checking convergence of K matrix
const float KMARGIN = 0.000001;
const int MAXCOMPARISONS = 100;

// constant variance estimates
//posVar: the distrust in the pos observation
//StateVar: the distrust in the initial state, which is the observation
//RandVar: the distrust in the current state
//in the future data about them and us might be different so we made different variances.
// More trust in the model/state (higher Posvar compared to RandVar) leads to a smoother but slower signal
// More trust in the observation (higher Randvar compared to PosVar) leads to a noiser but faster signal
const float posVar_us = 2;
const float stateVar_us = posVar_us;
const float randVar_us = 1;
const float posVar_them = 2;
const float stateVar_them = posVar_them;
const float randVar_them = 1;
const float posVar_ball = 1;
const float stateVar_ball = posVar_ball;
const float randVar_ball = 10;


const unsigned int INVALID_ID = - 1;

};

#endif //ROBOTEAM_WORLD_CONSTANTSK_H
