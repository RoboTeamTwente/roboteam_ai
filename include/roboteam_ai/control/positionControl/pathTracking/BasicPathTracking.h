//
// Created by ratoone on 12-11-19.
//

#ifndef RTT_BASICPATHTRACKING_H
#define RTT_BASICPATHTRACKING_H

#include "control/positionControl/PositionControlUtils.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Vector2.h"
#include "utilities/Constants.h"
#include "PathTrackingAlgorithm.h"

namespace rtt::ai::control {

/**
 * Path tracking algorithm. See method computePath for details.
 */
class BasicPathTracking : public PathTrackingAlgorithm {
   private:
    static constexpr double MAX_VELOCITY = Constants::MAX_VEL();

   public:
    /**
     * Generates an output velocity and angle according to the implemented algorithm.
     * After reaching a certain distance to the closest path point, it will go to the next one. <br><br>
     * BasicPathTracking just computes the desired velocity as the difference between the current point
     * and the next one.
     * @param currentPosition
     * @param currentVelocity
     * @param pathPoints the path as a list of points
     * @param outputVelocity control velocity that will be outputted to the robot at the current tick
     * @param outputAngle the resulting orientation of the robot at the current tick
     * @param robotId the ID of the current robot
     * @param angle the desired orientation angle of the robot - if omitted, the robot will face its velocity
     */
    Position trackPath(const Vector2 &currentPosition, const Vector2 &currentVelocity, std::vector<Vector2> &pathPoints, int robotId, double angle) override;
};
}  // namespace rtt::ai::control

#endif  // RTT_BASICPATHTRACKING_H
