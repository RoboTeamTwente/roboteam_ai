//
// Created by ratoone on 30-01-20.
//

#include "control/positionControl/PositionControlUtils.h"

namespace rtt::ai::control {

bool PositionControlUtils::isTargetChanged(const Vector2 &targetPos, const Vector2 &oldTarget) { return (targetPos - oldTarget).length() > MAX_TARGET_DEVIATION; }

bool PositionControlUtils::isTargetReached(const Vector2 &targetPos, const Vector2 &currentPosition) {
    return (targetPos - currentPosition).length() < MIN_DISTANCE_TARGET_REACHED;
}

void PositionControlUtils::removeFirstIfReached(std::vector<Vector2> &path, const Vector2 &currentPosition) {
    if (path.size() > 1 && isTargetReached(path.front(), currentPosition)) {
        path.erase(path.begin());
    }
}
}  // namespace rtt::ai::control
