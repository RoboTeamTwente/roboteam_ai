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
        static double constrainAngle(double angle);
};
}

#endif //ROBOTEAM_AI_CONTROLUTILS_H
