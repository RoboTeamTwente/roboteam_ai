//
// Created by ratoone on 24-01-20.
//

#ifndef RTT_NUMTREESTRACKING_H
#define RTT_NUMTREESTRACKING_H

#include <utilities/Constants.h>
#include <roboteam_utils/Vector2.h>
#include "PidTracking.h"

namespace rtt::ai::control{
class NumTreesTracking {
private:
    double maxVelocity = Constants::MAX_VEL();
    double minimumDistance = 2*Constants::ROBOT_RADIUS();
    unsigned long stepsAhead = 4;

    PidTracking pidTracking;
public:
    void trackPath(const rtt::Vector2 &currentPosition, const rtt::Vector2 &currentVelocity,
                   std::list<rtt::Vector2> &pathPoints, rtt::Vector2 &outputVelocity, double &outputAngle);
};

}


#endif //RTT_NUMTREESTRACKING_H
