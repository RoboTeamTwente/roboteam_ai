//
// Created by kjhertenberg on 16-5-19.
//

#include "roboteam_world/kalman/kalmanThem.h"

namespace rtt{

    kalmanThem::kalmanThem() {
        kalmanThem(99);
    }

    kalmanThem::kalmanThem(uint id) {
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
        this->R = {{posVar_them, 0, 0},
                   {0, posVar_them, 0},
                   {0, 0, posVar_them}};
        this->I.eye();
        this->P = {{stateVar_them, 0, 0, 0, 0, 0},
                   {0, stateVar_them, 0, 0, 0, 0},
                   {0, 0, stateVar_them, 0, 0, 0},
                   {0, 0, 0, stateVar_them, 0, 0},
                   {0, 0, 0, 0, stateVar_them, 0},
                   {0, 0, 0, 0, 0, stateVar_them}};
        this->Q = {{timeDiff * timeDiff * randVar_them, timeDiff * randVar_them, 0, 0, 0, 0},
                   {timeDiff * randVar_them, randVar_them, 0, 0, 0, 0},
                   {0, 0, timeDiff * timeDiff * randVar_them, timeDiff * randVar_them, 0, 0},
                   {0, 0, timeDiff * randVar_them, randVar_them, 0, 0},
                   {0, 0, 0, 0, timeDiff * timeDiff * randVar_them, timeDiff * randVar_them},
                   {0, 0, 0, 0, timeDiff * randVar_them, randVar_them}};
        this->K.zeros();
    }


}