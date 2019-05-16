//
// Created by kjhertenberg on 16-5-19.
//

#include "roboteam_world/kalman/kalmanBall.h"

namespace rtt {

    kalmanBall::kalmanBall(){
        kalmanBall(99);
    }

    kalmanBall::kalmanBall(uint id) {
        this->id = id;
        this->observationTimeStamp = -1.0;
        this->invisibleCounter = 0;
        this->exists = false;
        this->comparisonCount = 0;
        this->X.zeros();
        this->Z.zeros();
        this->F = {{1, timeDiff, 0, 0,        0, 0},
                   {0, 1,        0, 0,        0, 0},
                   {0, 0,        1, timeDiff, 0, 0},
                   {0, 0,        0, 1,        0, 0},
                   {0, 0,        0, 0,        1, timeDiff},
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
        this->Q = {{timeDiff * timeDiff * randVar_ball, timeDiff * randVar_ball, 0, 0, 0, 0},
                   {timeDiff * randVar_ball, randVar_ball, 0, 0, 0, 0},
                   {0, 0, timeDiff * timeDiff * randVar_ball, timeDiff * randVar_ball, 0, 0},
                   {0, 0, timeDiff * randVar_ball, randVar_ball, 0, 0},
                   {0, 0, 0, 0, timeDiff * timeDiff * randVar_ball, timeDiff * randVar_ball},
                   {0, 0, 0, 0, timeDiff * randVar_ball, randVar_ball}};
        this->K.zeros();
    }

}