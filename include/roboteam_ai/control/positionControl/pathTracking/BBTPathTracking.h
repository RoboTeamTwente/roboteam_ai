//
// Created by tijmen on 27-10-21.
//

#ifndef RTT_BBTPATHTRACKING_H
#define RTT_BBTPATHTRACKING_H

#include "PidTracking.h"
#include "control/positionControl/PositionControlUtils.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Vector2.h"
#include "utilities/Constants.h"

namespace rtt::ai::control {

/**
 * Path tracking algorithm for BBT
 */
class BBTPathTracking {
   private:
    static constexpr unsigned long STEPS_AHEAD = 1;
    PidTracking pidTracking;

   public:
    /**
     * Generates an output velocity and angle according to the implemented algorithm.
     * After reaching a certain distance to the closest path point, it will go to the next one. <br><br>
     * DensePathTracking looks at the target position after N steps and follows that
     * using a PID controller (the PidTracking). It is assumed that the path is dense.
     * @param currentPosition
     * @param currentVelocity
     * @param pathPoints the path as a list of points
     * @param velocityPoints the path as a list of velocities
     * @param robotId the ID of the current robot
     * @param angle the desired orientation angle of the robot - if omitted, the robot will face its velocity
     * @return a structure containing the tracking velocity and the orientation angle
     */
    Position trackPathForwardAngle(const Vector2 &currentPosition, const Vector2 &currentVelocity, std::vector<std::pair<Vector2, Vector2>> &pathVelocityPoints, int robotId,
                                   stp::PIDType pidType);
};

}  // namespace rtt::ai::control

#endif  // RTT_BBTPATHTRACKING_H
