//
// Created by ratoone on 12-11-19.
//

#ifndef RTT_BASICPATHTRACKING_H
#define RTT_BASICPATHTRACKING_H


#include <utilities/Constants.h>
#include <roboteam_utils/Vector2.h>

class BasicPathTracking {
private:
    double maxVelocity = rtt::ai::Constants::MAX_VEL();
    double minimumDistance = 2*rtt::ai::Constants::ROBOT_RADIUS();

public:
    void trackPath(const rtt::Vector2 &currentPosition, const rtt::Vector2 &currentVelocity,
            std::list<rtt::Vector2> &pathPoints, rtt::Vector2 &outputVelocity, double &outputAngle);
};


#endif //RTT_BASICPATHTRACKING_H
