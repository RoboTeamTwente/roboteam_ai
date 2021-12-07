//
// Created by ratoone on 24-01-20.
//

#include "control/positionControl/pathTracking/DensePathTracking.h"

#include "control/positionControl/PositionControlUtils.h"

namespace rtt::ai::control {

Position DensePathTracking::trackPath(const Vector2 &currentPosition, const Vector2 &currentVelocity, std::vector<Vector2> &pathPoints, int robotId, double angle,
                                      stp::PIDType pidType) {
    int lookAhead = std::min(pathPoints.size(), STEPS_AHEAD);
    Vector2 currentTarget = *std::next(pathPoints.begin(), lookAhead - 1);
    if (pathPoints.size() > 1 && PositionControlUtils::isTargetReached(currentTarget, currentPosition)) {
        // track the Nth point, or the last if the size is smaller than N; the untracked ones are discarded
        pathPoints.erase(pathPoints.begin(), std::next(pathPoints.begin(), lookAhead));
    }

    std::vector<Vector2> tempPath{currentTarget};
    return pidTracking.trackPath(currentPosition, currentVelocity, tempPath, robotId, angle, pidType);
}
}  // namespace rtt::ai::control