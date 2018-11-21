//
// Created by baris on 16/11/18.
//

#ifndef ROBOTEAM_AI_CONTROLUTILS_H
#define ROBOTEAM_AI_CONTROLUTILS_H

#include "../utilities/Constants.h"
#include "math.h"

namespace control {
class ControlUtils {
    public:
        static double calculateAngularVelocity(double robotAngle, double targetAngle);
};
}

#endif //ROBOTEAM_AI_CONTROLUTILS_H
