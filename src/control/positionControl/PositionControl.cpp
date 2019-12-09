//
// Created by ratoone on 18-11-19.
//

#include <control/positionControl/PositionControl.h>
#include <interface/api/Input.h>

using namespace rtt::ai;

PositionControl::PositionControl(double fieldWidth, double fieldLength,
                                 const std::vector<rtt::Vector2 *> &robotPositions) {
    pathPlanningAlgorithm = VoronoiPathPlanning(fieldWidth, fieldLength, robotPositions);
}

//TODO: add collision sensing
RobotCommand
PositionControl::computeAndTrackPath(const std::vector<std::shared_ptr<ai::world::Robot>>&robots,
        int robotId, const Vector2 &currentPosition,
        const Vector2 &currentVelocity, const Vector2 &targetPosition) {
    if (computedPaths.find(robotId) == computedPaths.end()) {
        computedPaths.insert({robotId, std::list<Vector2>()});
    }

    if (computedPaths[robotId].empty() || computedPaths[robotId].back() != targetPosition) {
        std::vector<Vector2*> robotPositions(robots.size());
        std::transform(robots.begin(), robots.end(), robotPositions.begin(),
                       [](auto robot)-> Vector2* {return &(robot->pos);});
        pathPlanningAlgorithm.setRobotPositions(robotPositions);
        computedPaths[robotId] = pathPlanningAlgorithm.computePath(currentPosition, targetPosition);
    }

    RobotCommand command = RobotCommand();
    double angle;
    command.pos = computedPaths[robotId].front();
    pathTrackingAlgorithm.trackPath(
            currentPosition,
            currentVelocity,
            computedPaths[robotId],
            command.vel,
            angle);
    command.angle = angle;

    for (const auto& point: computedPaths[robotId]) {
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

