//
// Created by kjhertenberg on 16-5-19.
//

#include "roboteam_world/kalman/kalmanUs.h"

namespace rtt{

    kalmanUs::kalmanUs() {
        kalmanUs(99);
    }

    kalmanUs::kalmanUs(uint id) {
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
        this->R = {{posVar_us, 0, 0},
                   {0, posVar_us, 0},
                   {0, 0, posVar_us}};
        this->I.eye();
        this->P = {{stateVar_us, 0, 0, 0, 0, 0},
                   {0, stateVar_us, 0, 0, 0, 0},
                   {0, 0, stateVar_us, 0, 0, 0},
                   {0, 0, 0, stateVar_us, 0, 0},
                   {0, 0, 0, 0, stateVar_us, 0},
                   {0, 0, 0, 0, 0, stateVar_us}};
        this->Q = {{timeDiff * timeDiff * randVar_us, timeDiff * randVar_us, 0, 0, 0, 0},
                   {timeDiff * randVar_us, randVar_us, 0, 0, 0, 0},
                   {0, 0, timeDiff * timeDiff * randVar_us, timeDiff * randVar_us, 0, 0},
                   {0, 0, timeDiff * randVar_us, randVar_us, 0, 0},
                   {0, 0, 0, 0, timeDiff * timeDiff * randVar_us, timeDiff * randVar_us},
                   {0, 0, 0, 0, timeDiff * randVar_us, randVar_us}};
        this->K.zeros();
    }


}