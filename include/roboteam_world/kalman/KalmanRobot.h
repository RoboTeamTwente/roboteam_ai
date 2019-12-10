//
// Created by kjhertenberg on 16-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANUS_H
#define ROBOTEAM_WORLD_KALMANUS_H

#include "KalmanObject.h"

namespace world {

class KalmanRobot : public KalmanObject {
public:
    KalmanRobot();
    KalmanRobot(uint id);
};
}

#endif //ROBOTEAM_WORLD_KALMANUS_H
