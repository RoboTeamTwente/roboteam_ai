//
// Created by baris on 16/11/18.
//

#ifndef ROBOTEAM_AI_CONTROLUTILS_H
#define ROBOTEAM_AI_CONTROLUTILS_H

#include "../utilities/Constants.h"
#include "roboteam_utils/Vector2.h"
#include "math.h"

typedef rtt::Vector2 Vec;
namespace control {
class ControlUtils {
    public:
        static double calculateAngularVelocity(double robotAngle, double targetAngle);
        static double TriangleArea(Vec A,Vec B,Vec C);
        static bool pointInTriangle(Vec PointToCheck,Vec TP1, Vec TP2, Vec TP3);
        static bool pointInRectangle(Vec PointToCheck,Vec SP1, Vec SP2, Vec SP3,Vec SP4);
        static double constrainAngle(double angle);
        static double distanceToLine(Vec PointToCheck, Vec LineStart, Vec LineEnd);
        static double angleDifference(double A1, double A2);
};

}


#endif //ROBOTEAM_AI_CONTROLUTILS_H
