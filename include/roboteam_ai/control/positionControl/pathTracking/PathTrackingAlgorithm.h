//
// Created by ratoone on 09-03-20.
//

#ifndef RTT_PATHTRACKINGALGORITHM_H
#define RTT_PATHTRACKINGALGORITHM_H

#include <roboteam_utils/Position.h>

namespace rtt::ai::control{
class PathTrackingAlgorithm {
public:
    virtual Position trackPath(const Vector2 &currentPosition, const Vector2 &currentVelocity, std::vector<Vector2> &pathPoints, int robotId, double angle) = 0;

    /**
     * Uses the implementation of trackPath, but replaces the angle with the orientation of the
     * direction of the current position and the next point in the path
     * @param currentPosition
     * @param currentVelocity
     * @param pathPoints
     * @param robotId
     * @return a structure containing the tracking velocity and the orientation angle
     */
    Position trackPathDefaultAngle(const Vector2 &currentPosition, const Vector2 &currentVelocity, std::vector<Vector2> &pathPoints, int robotId);
};
}

#endif //RTT_PATHTRACKINGALGORITHM_H
