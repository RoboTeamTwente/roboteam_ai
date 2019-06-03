//
// Created by kjhertenberg on 16-5-19.
//

#include "roboteam_world/kalman/kalmanUs.h"

namespace rtt{

    kalmanUs::kalmanUs() : kalmanUs(99){
    }

    kalmanUs::kalmanUs(uint id) {
        //in the future data about them and us might be different so we made different classes
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
                   {0, 0,        1, TIMEDIFF},
                   {0, 0,        0, 1       }};
        this->H = {{1, 0, 0, 0},
                   {0, 0, 1, 0}};
        this->R = {{posVar_us, 0},
                   {0, posVar_us}};
        this->I.eye();
        this->P = {{stateVar_us, 0, 0, 0},
                   {0, stateVar_us, 0, 0},
                   {0, 0, stateVar_us, 0},
                   {0, 0, 0, stateVar_us}};
        arma::fmat::fixed<STATEINDEX, 2> tempQ = {{TIMEDIFF, 0},
                                                  {1, 0},
                                                  {0, TIMEDIFF},
                                                  {0, 1}};
        arma::fmat::fixed<2, STATEINDEX> tempQ_t = tempQ.t();
        this->Q = tempQ * tempQ_t * randVar_us;
        this->K.zeros();
    }


}