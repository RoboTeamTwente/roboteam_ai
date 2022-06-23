//
// Created by ratoone on 30-01-20.
//

#include "control/positionControl/PositionControlUtils.h"

#include "utilities/Constants.h"

namespace rtt::ai::control {

bool PositionControlUtils::isTargetChanged(const Vector2 &targetPos, const Vector2 &oldTarget) {
    return (targetPos - oldTarget).length() > Constants::POSITION_CONTROL_MAX_TARGET_DEVIATION();
}

bool PositionControlUtils::isTargetReached(const Vector2 &targetPos, const Vector2 &currentPosition) {
    return (targetPos - currentPosition).length() < Constants::POSITION_CONTROL_MIN_DISTANCE_REACHED();
}
bool PositionControlUtils::isMoving(const Vector2 &velocity) { return velocity.length() > Constants::POSITION_CONTROL_MAX_STILL_VELOCITY(); }
}  // namespace rtt::ai::control
