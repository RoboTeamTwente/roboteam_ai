//
// Created by ratoone on 18-11-19.
//

#include "control/positionControl/PositionControl.h"
#include "interface/api/Input.h"

namespace rtt::ai::control {

// TODO: add projection to outside defence area (project target position)(is this really needed?)
RobotCommand PositionControl::computeAndTrackPath(const world::Field &field, int robotId, const Vector2 &currentPosition, const Vector2 &currentVelocity,
                                                  const Vector2 &targetPosition) {
    // TODO: this is a workaround caused by the fact that the field is not global
    collisionDetector.setField(field);
    if (shouldRecalculatePath(currentPosition, targetPosition, robotId)) {
        computedPaths[robotId] = pathPlanningAlgorithm.computePath(currentPosition, targetPosition);
    }

    interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::green, robotId, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::blue, robotId, interface::Drawing::DOTS);

    RobotCommand command = RobotCommand();
    command.pos = computedPaths[robotId].front();
    Position trackingVelocity = pathTrackingAlgorithm.trackPath(currentPosition, currentVelocity, computedPaths[robotId]);
    command.vel = Vector2(trackingVelocity.x, trackingVelocity.y);
    command.angle = trackingVelocity.rot;

    return command;
}

bool PositionControl::shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos, int robotId) {
    return computedPaths[robotId].empty() || PositionControlUtils::isTargetChanged(targetPos, computedPaths[robotId].back()) ||
           collisionDetector.isRobotCollisionBetweenPoints(currentPosition, computedPaths[robotId].front());
}

void PositionControl::setRobotVector(const std::vector<world_new::view::RobotView> &robots) { collisionDetector.setRobotVector(robots); }
}  // namespace rtt::ai::control