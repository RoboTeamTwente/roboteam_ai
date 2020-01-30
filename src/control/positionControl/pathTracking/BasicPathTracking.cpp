//
// Created by ratoone on 12-11-19.
//

#include "control/positionControl/pathTracking/BasicPathTracking.h"

namespace rtt::ai::control{

Position BasicPathTracking::trackPath(const Vector2 &currentPosition, const Vector2 &currentVelocity,
        std::vector<Vector2> &pathPoints) {
    PositionControlUtils::removeFirstIfReached(pathPoints, currentPosition);

    Vector2 velocity = pathPoints.front() - currentPosition;

    if (velocity.length() > MAX_VELOCITY) {
        velocity = velocity.stretchToLength(MAX_VELOCITY);
    }

    return Position(velocity, velocity.angle());
}
}