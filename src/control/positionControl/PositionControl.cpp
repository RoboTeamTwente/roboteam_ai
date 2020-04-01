//
// Created by ratoone on 18-11-19.
//

#include <roboteam_utils/Print.h>
#include "control/positionControl/PositionControl.h"
#include "control/positionControl/PositionControlUtils.h"
#include "interface/api/Input.h"


namespace rtt::ai::control {
RobotCommand PositionControl::computeAndTrackPath(const world::Field &field, int robotId, const Vector2 &currentPosition, const Vector2 &currentVelocity,
                                                  const Vector2 &targetPosition) {
    collisionDetector.setField(field);
    // if the target position is outside of the field (i.e. bug in AI), do nothing
    if(!collisionDetector.isPointInsideField(targetPosition)){
        RTT_ERROR("Point sent is outside field: ")
        return {};
    }

    if (shouldRecalculatePath(currentPosition, targetPosition, currentVelocity, robotId)) {
        computedPaths[robotId] = pathPlanningAlgorithm.computePath(currentPosition, targetPosition);
    }

    interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::green, robotId, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::PATHFINDING, {computedPaths[robotId].front(), currentPosition}, Qt::green, robotId, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::blue, robotId, interface::Drawing::DOTS);

    RobotCommand command = RobotCommand();
    command.pos = computedPaths[robotId].front();
    Position trackingVelocity = pathTrackingAlgorithm.trackPathDefaultAngle(currentPosition, currentVelocity, computedPaths[robotId], robotId);
    command.vel = Vector2(trackingVelocity.x, trackingVelocity.y);
    command.angle = trackingVelocity.rot;

    return command;
}

bool PositionControl::shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos,
                                            const Vector2 &currentVelocity, int robotId) {
    return computedPaths[robotId].empty() || PositionControlUtils::isTargetChanged(targetPos, computedPaths[robotId].back()) ||
            (currentVelocity != Vector2(0, 0) && collisionDetector.isCollisionBetweenPoints(currentPosition, computedPaths[robotId].front()));
}

void PositionControl::setRobotPositions(std::vector<Vector2> &robotPositions) {
    collisionDetector.setRobotPositions(robotPositions); }
}  // namespace rtt::ai::control