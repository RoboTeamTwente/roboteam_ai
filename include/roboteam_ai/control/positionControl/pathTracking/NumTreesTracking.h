//
// Created by ratoone on 24-01-20.
//

#ifndef RTT_NUMTREESTRACKING_H
#define RTT_NUMTREESTRACKING_H

#include "utilities/Constants.h"
#include "roboteam_utils/Vector2.h"
#include "PidTracking.h"
#include "roboteam_utils/Position.h"
#include "control/positionControl/PositionControlUtils.h"

namespace rtt::ai::control{

/**
 * Path tracking algorithm. See method computePath for details.
 */
class NumTreesTracking {
private:
    static constexpr unsigned long STEPS_AHEAD = 4;

    PidTracking pidTracking;
public:
    /**
     * Generates an output velocity and angle according to the implemented algorithm.
     * After reaching a certain distance to the closest path point, it will go to the next one. <br><br>
     * NumTreesTracking looks at the target position after N steps and follows that
     * using a PID controller (the PidTracking).
     * @param currentPosition
     * @param currentVelocity
     * @param pathPoints the path as a list of points
     * @param outputVelocity control velocity that will be outputted to the robot at the current tick
     * @param outputAngle the resulting orientation of the robot at the current tick
     */
    Position trackPath(const Vector2 &currentPosition, const Vector2 &currentVelocity, std::vector<Vector2> &pathPoints);
};

}


#endif //RTT_NUMTREESTRACKING_H
