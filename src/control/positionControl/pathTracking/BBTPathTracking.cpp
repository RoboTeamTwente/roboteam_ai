//
// Created by tijmen on 27-10-21.
//

#include "control/positionControl/pathTracking/BBTPathTracking.h"

#include <stp/StpInfo.h>

#include "roboteam_utils/Print.h"

namespace rtt::ai::control {

Position BBTPathTracking::trackPathForwardAngle(const Vector2 &currentPosition, const Vector2 &currentVelocity, std::vector<std::pair<Vector2, Vector2>> &pathVelocityPoints,
                                                int robotId, stp::PIDType pidType) {
    int lookAhead = std::min(pathVelocityPoints.size(), STEPS_AHEAD);
    Vector2 currentTarget = std::next(pathVelocityPoints.begin(), lookAhead - 1)->first;
    Vector2 currentTargetVelocity = std::next(pathVelocityPoints.begin(), lookAhead - 1)->second;
    if (pathVelocityPoints.size() > 1 && PositionControlUtils::isTargetReached(currentTarget, currentPosition)) {
        // track the Nth point, or the last if the size is smaller than N; the untracked ones are discarded
        pathVelocityPoints.erase(pathVelocityPoints.begin(), std::next(pathVelocityPoints.begin(), lookAhead));
    }

    std::vector<Vector2> tempPath{currentTarget};
    Position pidPosition = pidTracking.trackPath(currentPosition, currentVelocity, tempPath, robotId, 0, pidType);
    Vector2 pidVelocity{pidPosition.x, pidPosition.y};
    if (pathVelocityPoints.size() > 2) {
        pidVelocity = pidVelocity.stretchToLength(currentTargetVelocity.length());
    }
    Position returnPosition{pidVelocity.x, pidVelocity.y, (currentTarget - currentPosition).angle()};
    return returnPosition;
}
}  // namespace rtt::ai::control