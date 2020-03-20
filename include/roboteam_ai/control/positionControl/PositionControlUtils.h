//
// Created by ratoone on 30-01-20.
//

#ifndef RTT_POSITIONCONTROLUTILS_H
#define RTT_POSITIONCONTROLUTILS_H

#include "roboteam_utils/Vector2.h"
#include "utilities/Constants.h"

namespace rtt::ai::control {

class PositionControlUtils {
   private:
    static constexpr double MAX_TARGET_DEVIATION = 0.05;

    // minimum distance needed to consider the current target reached
    static constexpr double MIN_DISTANCE_TARGET_REACHED = 2 * Constants::ROBOT_RADIUS();

   public:
    /**
     * If the distance between the old target and the new target > MAX_TARGET_DEVIATION
     * @param targetPos
     * @param oldTarget
     * @return
     */
    static bool isTargetChanged(const Vector2 &targetPos, const Vector2 &oldTarget);

    /**
     * The target is considered to be reached if the distance between target and currentPosition < MIN_DISTANCE
     * @param targetPos
     * @param currentPosition
     * @return
     */
    static bool isTargetReached(const Vector2 &targetPos, const Vector2 &currentPosition);

    /**
     * Removes the first element in the path in the case the first point is reached
     * @param path the vector of path points
     * @param currentPosition the current position of the robot
     */
    static void removeFirstIfReached(std::vector<Vector2> &path, const Vector2 &currentPosition);
};
}  // namespace rtt::ai::control

#endif  // RTT_POSITIONCONTROLUTILS_H
