//
// Created by ratoone on 18-11-19.
//

#include "control/positionControl/PositionControl.h"
#include "interface/api/Input.h"
#include "control/positionControl/pathTracking/NumTreesTracking.h"

namespace rtt::ai::control {

PositionControl::PositionControl(const std::vector<world_new::robot::Robot> &robots): robots(robots) {
    collisionDetector = new CollisionDetector(robots);
    pathPlanningAlgorithm = new NumTreesPlanning(*collisionDetector);
    pathTrackingAlgorithm = new NumTreesTracking();
}

//TODO: add projection to outside defence area (project target position)(is this really needed?)
RobotCommand
PositionControl::computeAndTrackPath(world::Field *field, int robotId, const Vector2 &currentPosition,
                                     const Vector2 &currentVelocity, const Vector2 &targetPosition) {
    //TODO: this is a workaround caused by the fact that the field is not global
    collisionDetector->setField(field);
    if (shouldRecalculatePath(currentPosition, targetPosition, robotId)) {
        computedPaths[robotId] = pathPlanningAlgorithm->computePath(currentPosition, targetPosition);
    }

    RobotCommand command = RobotCommand();
    double angle;
    command.pos = computedPaths[robotId].front();
    pathTrackingAlgorithm->trackPath(
            currentPosition,
            currentVelocity,
            computedPaths[robotId],
            command.vel,
            angle);
    command.angle = angle;

    std::vector<Vector2> path(computedPaths[robotId].begin(),computedPaths[robotId].end());
    interface::Input::drawData(interface::Visual::PATHFINDING, path, Qt::green, robotId, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::PATHFINDING, path, Qt::blue, robotId, interface::Drawing::DOTS);

    return command;
}

bool PositionControl::shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos, int robotId) {
    return computedPaths[robotId].empty() ||
            // distance between the new target and the former target
            (targetPos - computedPaths[robotId].back()).length() > MAX_DEVIATION ||
            collisionDetector->isRobotCollisionBetweenPoints(currentPosition, computedPaths[robotId].front());

}

}