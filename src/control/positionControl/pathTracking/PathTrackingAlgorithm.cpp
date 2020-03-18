//
// Created by ratoone on 09-03-20.
//

#include "control/positionControl/pathTracking/PathTrackingAlgorithm.h"

namespace rtt::ai::control{
    Position control::PathTrackingAlgorithm::trackPathDefaultAngle(const Vector2 &currentPosition, const Vector2 &currentVelocity,
                                                                   std::vector<Vector2> &pathPoints, int robotId) {
        auto position = trackPath(currentPosition, currentVelocity, pathPoints, robotId, 0);
        Vector2 velocity = {position.x, position.y};
        return {velocity, (pathPoints.front() - currentPosition).angle()};
    }
}