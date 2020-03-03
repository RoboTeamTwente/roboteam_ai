//
// Created by ratoone on 24-01-20.
//

#ifndef RTT_DENSEPATHTRACKING_H
#define RTT_DENSEPATHTRACKING_H

#include "PidTracking.h"
#include "control/positionControl/PositionControlUtils.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Vector2.h"
#include "utilities/Constants.h"

namespace rtt::ai::control {

/**
 * Path tracking algorithm. See method computePath for details.
 */
class DensePathTracking {
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
     * @param outputVelocity control velocity that will be outputted to the robot at the current tick
     * @param outputAngle the resulting orientation of the robot at the current tick
     */
    Position trackPath(const Vector2 &currentPosition, const Vector2 &currentVelocity, std::vector<Vector2> &pathPoints);
};

}  // namespace rtt::ai::control

#endif  //RTT_DENSEPATHTRACKING_H
