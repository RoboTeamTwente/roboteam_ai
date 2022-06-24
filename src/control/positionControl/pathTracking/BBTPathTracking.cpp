//
// Created by tijmen on 27-10-21.
//

#include "control/positionControl/pathTracking/BBTPathTracking.h"

#include <stp/StpInfo.h>

#include <span>

#include "roboteam_utils/Print.h"

namespace rtt::ai::control {

Position BBTPathTracking::trackPathForwardAngle(const Vector2 &currentPosition, const Vector2 &currentVelocity, stp::PIDType pidType) {
    interface::Input::drawData(interface::Visual::PATHFINDING, remainingPath, Qt::yellow, robotId, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::PATHFINDING, remainingPath, Qt::green, robotId, interface::Drawing::DOTS);

    if (remainingPath.size() > ai::Constants::POSITION_CONTROL_STEP_COUNT()) {
        interface::Input::drawData(interface::Visual::PATHFINDING, {remainingPath[ai::Constants::POSITION_CONTROL_STEP_COUNT()].position}, Qt::red, robotId,
                                   interface::Drawing::DOTS);
    }

    if (remainingPath.empty()) {
        return {0, 0, 0};
    }

    auto lookAhead = std::min(remainingPath.size(), STEPS_AHEAD);
    const auto &currentTarget = std::next(remainingPath.begin(), lookAhead - 1);
    if (PositionControlUtils::isTargetReached(currentTarget->position, currentPosition)) {
        // Track the Nth point, or the last if the size is smaller than N; the untracked ones are discarded
        remainingPath = remainingPath.subspan(lookAhead);
    }

    const auto newPid = PositionControlUtils::getPIDValue(pidType);
    xPID.setPID(newPid);
    yPID.setPID(newPid);

    auto pidVelocity =
        Vector2{
            xPID.getOutput(currentPosition.x, currentTarget->position.x),
            yPID.getOutput(currentPosition.y, currentTarget->position.y),
        }
            .stretchToLength(currentTarget->velocity.length());

    return {pidVelocity.x, pidVelocity.y, (currentTarget->position - currentPosition).angle()};
}

UpdatePath BBTPathTracking::shouldUpdatePath(const Vector2 &currentPos, const Vector2 &targetPos, const stp::AvoidObjects &avoidObjects) {
    if (!remainingPath.empty() && PositionControlUtils::isTargetChanged(targetPos, remainingPath.back().position)) return UPDATE_TARGET_CHANGED;
    if (remainingPath.empty() && !PositionControlUtils::isTargetReached(targetPos, currentPos)) return UPDATE_TARGET_REACHED;

    auto collision = collisionDetector.getFirstCollision(remainingPath, robotId, avoidObjects);
    if (collision.has_value() && remainingPath.size() > 2) return UPDATE_COLLISION_DETECTED;

    return DONT_UPDATE;
}

void BBTPathTracking::updatePath(std::vector<BB::PosVelVector> &&newPath) {
    path = std::move(newPath);
    remainingPath = std::span(path);
}
BBTPathTracking::BBTPathTracking(int robotId, const CollisionDetector &collisionDetector) : robotId(robotId), collisionDetector(collisionDetector) {
    xPID.setMaxIOutput(Constants::MAX_VEL());
    yPID.setMaxIOutput(Constants::MAX_VEL());
}

std::span<const BB::PosVelVector> BBTPathTracking::getRemainingPath() { return remainingPath; }

}  // namespace rtt::ai::control