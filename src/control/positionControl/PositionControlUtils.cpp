//
// Created by ratoone on 30-01-20.
//

#include "control/positionControl/PositionControlUtils.h"

namespace rtt::ai::control {

bool PositionControlUtils::isTargetChanged(const Vector2 &targetPos, const Vector2 &oldTarget) { return (targetPos - oldTarget).length() > MAX_TARGET_DEVIATION; }

bool PositionControlUtils::isTargetReached(const Vector2 &targetPos, const Vector2 &currentPosition) { return (targetPos - currentPosition).length() < Constants::ROBOT_RADIUS(); }
bool PositionControlUtils::isAroundTarget(const Vector2 &targetPos, const Vector2 &currentPosition) {
    return (targetPos - currentPosition).length() < 4 * Constants::ROBOT_RADIUS();
}
bool PositionControlUtils::isMoving(const Vector2 &velocity) { return velocity.length() > 0.05; }
}  // namespace rtt::ai::control
