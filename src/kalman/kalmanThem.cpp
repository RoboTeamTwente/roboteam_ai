//
// Created by kjhertenberg on 16-5-19.
//

#include "roboteam_world/kalman/kalmanThem.h"

namespace rtt{

    kalmanThem::kalmanThem() :kalmanThem(INVALID_ID){

    }

    kalmanThem::kalmanThem(uint id) {
        this->id = id;
        this->observationTimeStamp = -1.0;
        this->invisibleCounter = 0;
        this->exists = false;
        this->comparisonCount = 0;
        this->orientation = 0;
        this->omega = 0;
        this->X.zeros();
        this->Z.zeros();
        this->F = {{1, TIMEDIFF, 0, 0       },
                   {0, 1,        0, 0       },
                   {0, 0,        1, TIMEDIFF}};
        this->H = {{1, 0, 0, 0},
                   {0, 0, 1, 0}};
        this->R = {{posVar_them, 0},
                   {0, posVar_them}};
        this->I.eye();
        this->P = {{stateVar_them, 0, 0, 0},
                   {0, stateVar_them, 0, 0},
                   {0, 0, stateVar_them, 0},
                   {0, 0, 0, stateVar_them}};
        this->Q = {{TIMEDIFF * TIMEDIFF * randVar_them, TIMEDIFF * randVar_them, 0, 0},
                   {TIMEDIFF * randVar_them, randVar_them, 0, 0},
                   {0, 0, TIMEDIFF * TIMEDIFF * randVar_them, TIMEDIFF * randVar_them},
                   {0, 0, TIMEDIFF * randVar_them, randVar_them}};
        this->K.zeros();
    }


}