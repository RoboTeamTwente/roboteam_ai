//
// Created by ratoone on 18-11-19.
//

#include "control/positionControl/PositionControl.h"
#include "interface/api/Input.h"
#include "control/positionControl/pathTracking/NumTreesTracking.h"

namespace rtt::ai::control {

PositionControl::PositionControl(const std::vector<world_new::robot::Robot> &robots): robots(robots) {
    collisionDetector = std::make_unique<CollisionDetector>(robots);
    pathPlanningAlgorithm = std::make_unique<NumTreesPlanning>(*collisionDetector);
    pathTrackingAlgorithm = std::make_unique<NumTreesTracking>();
}

//TODO: add projection to outside defence area (project target position)(is this really needed?)
RobotCommand
PositionControl::computeAndTrackPath(world::Field &field, int robotId, const Vector2 &currentPosition,
                                     const Vector2 &currentVelocity, const Vector2 &targetPosition) {
    //TODO: this is a workaround caused by the fact that the field is not global
    collisionDetector->setField(field);
    if (shouldRecalculatePath(currentPosition, targetPosition, robotId)) {
        computedPaths[robotId] = pathPlanningAlgorithm->computePath(currentPosition, targetPosition);
    }

    interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::green, robotId, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::blue, robotId, interface::Drawing::DOTS);

    RobotCommand command = RobotCommand();
    command.pos = computedPaths[robotId].front();
    Position trackingVelocity = pathTrackingAlgorithm->trackPath(currentPosition, currentVelocity,computedPaths[robotId]);
    command.vel = Vector2(trackingVelocity.x, trackingVelocity.y);
    command.angle = trackingVelocity.rot;

    return command;
}

bool PositionControl::shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos, int robotId) {
    return computedPaths[robotId].empty() ||
            // distance between the new target and the former target
            (targetPos - computedPaths[robotId].back()).length() > MAX_TARGET_DEVIATION ||
           collisionDetector->isRobotCollisionBetweenPoints(currentPosition, computedPaths[robotId].front());
}
}