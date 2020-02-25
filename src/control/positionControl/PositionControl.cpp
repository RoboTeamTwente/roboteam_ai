//
// Created by ratoone on 18-11-19.
//

#include "control/positionControl/PositionControl.h"
#include "control/positionControl/PositionControlUtils.h"
#include "control/positionControl/pathTracking/NumTreesTracking.h"
#include "interface/api/Input.h"

namespace rtt::ai::control {
RobotCommand PositionControl::computeAndTrackPath(const world::Field &field, int robotId, const Vector2 &currentPosition, const Vector2 &currentVelocity,
                                                  const Vector2 &targetPosition) {
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
            collisionDetector.getRobotCollisionBetweenPoints(currentPosition, computedPaths[robotId].front());
}

void PositionControl::setRobotPositions(std::vector<Vector2> &robotPositions) {
    collisionDetector.setRobotPositions(robotPositions); }
}  // namespace rtt::ai::control