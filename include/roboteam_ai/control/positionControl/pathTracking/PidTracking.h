//
// Created by ratoone on 24-01-20.
//

#ifndef RTT_PIDTRACKING_H
#define RTT_PIDTRACKING_H

#include <utilities/Constants.h>
#include <roboteam_utils/Vector2.h>
#include <interface/api/Output.h>
#include <roboteam_utils/pid.h>

namespace rtt::ai::control{

class PidTracking {
private:
    double maxVelocity = rtt::ai::Constants::MAX_VEL();
    double minimumDistance = 2*rtt::ai::Constants::ROBOT_RADIUS();

    PID xPid = PID(0,0,0,0);
    PID yPid = PID(0,0,0,0);

public:
    PidTracking();

    void trackPath(const rtt::Vector2 &currentPosition, const rtt::Vector2 &currentVelocity,
                   std::list<rtt::Vector2> &pathPoints, rtt::Vector2 &outputVelocity, double &outputAngle);

    void updatePidValues();
};

}


#endif //RTT_PIDTRACKING_H
