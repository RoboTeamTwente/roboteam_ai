//
// Created by ratoone on 18-11-19.
//

#include "control/positionControl/PositionControl.h"

namespace rtt::ai::control {

PositionControl::PositionControl(world::World& world, world::Field& field): world(world), field(field) {
    collisionDetector = new CollisionDetector(world, field);
    pathPlanningAlgorithm = new NumTreesPlanning(*collisionDetector);
    pathTrackingAlgorithm = new BasicPathTracking();
}

//TODO: add projection to outside defence area (project target position)(is this really needed?)
RobotCommand PositionControl::computeAndTrackPath(int robotId, const Vector2 &currentPosition,
        const Vector2 &currentVelocity, const Vector2 &targetPosition) {

    if (shouldRecalculatePath(currentPosition, targetPosition, robotId)) {
        auto robots = world.getAllRobots();
        std::vector<Vector2 *> robotPositions(robots.size());
        std::transform(robots.begin(), robots.end(), robotPositions.begin(),
                       [](auto robot) -> Vector2 * { return &(robot->pos); });
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

    for (const auto &point: computedPaths[robotId]) {
        rtt::ai::interface::Input::drawData(rtt::ai::interface::Visual::PATHFINDING_DEBUG, {point}, Qt::green,
                                            robotId,
                                            rtt::ai::interface::Drawing::DOTS, 12, 12);
        interface::Input::drawData(interface::Visual::PATHFINDING_DEBUG, {point, point + command.vel * 0.4},
                                   Qt::red,
                                   robotId,
                                   interface::Drawing::LINES_CONNECTED);
    }
    return command;
}

bool PositionControl::shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos, int robotId) {
    return computedPaths[robotId].empty() ||
            // distance between the new target and the former target
            (targetPos - computedPaths[robotId].back()).length() > MAX_DEVIATION ||
            collisionDetector->isRobotCollisionBetweenPoints(currentPosition, computedPaths[robotId].front());

}

}