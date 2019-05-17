//
// Created by kjhertenberg on 16-5-19.
//

#include "roboteam_world/kalman/kalmanBall.h"

namespace rtt {
    kalmanBall::kalmanBall() {
        this->id=INVALID_ID;
        this->observationTimeStamp = -1.0;
        this->invisibleCounter = 0;
        this->exists = false;
        this->comparisonCount = 0;
        this->X.zeros();
        this->Z.zeros();
        this->F = {{1, TIMEDIFF, 0, 0,        0, 0},
                   {0, 1,        0, 0,        0, 0},
                   {0, 0,        1, TIMEDIFF, 0, 0},
                   {0, 0,        0, 1,        0, 0},
                   {0, 0,        0, 0,        1, TIMEDIFF},
                   {0, 0,        0, 0,        0, 1}};
        this->H = {{1, 0, 0, 0, 0, 0},
                   {0, 0, 1, 0, 0, 0},
                   {0, 0, 0, 0, 1, 0}};
        this->R = {{posVar_ball, 0, 0},
                   {0, posVar_ball, 0},
                   {0, 0, posVar_ball}};
        this->I.eye();
        this->P = {{stateVar_ball, 0, 0, 0, 0, 0},
                   {0, stateVar_ball, 0, 0, 0, 0},
                   {0, 0, stateVar_ball, 0, 0, 0},
                   {0, 0, 0, stateVar_ball, 0, 0},
                   {0, 0, 0, 0, stateVar_ball, 0},
                   {0, 0, 0, 0, 0, stateVar_ball}};
        this->Q = {{TIMEDIFF * TIMEDIFF * randVar_ball, TIMEDIFF * randVar_ball, 0, 0, 0, 0},
                   {TIMEDIFF * randVar_ball, randVar_ball, 0, 0, 0, 0},
                   {0, 0, TIMEDIFF * TIMEDIFF * randVar_ball, TIMEDIFF * randVar_ball, 0, 0},
                   {0, 0, TIMEDIFF * randVar_ball, randVar_ball, 0, 0},
                   {0, 0, 0, 0, TIMEDIFF * TIMEDIFF * randVar_ball, TIMEDIFF * randVar_ball},
                   {0, 0, 0, 0, TIMEDIFF * randVar_ball, randVar_ball}};
        this->K.zeros();
    }

}