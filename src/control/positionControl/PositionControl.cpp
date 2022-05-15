//
// Created by ratoone on 18-11-19.
//

#include "control/positionControl/PositionControl.h"

#include <span>

#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"

namespace rtt::ai::control {

PositionControlCommand PositionControl::computeAndTrackTrajectory(const rtt::world::Field &field, int robotId, Vector2 currentPosition, Vector2 currentVelocity,
                                                                  Vector2 targetPosition, double maxRobotVelocity, stp::PIDType pidType, stp::AvoidObjects avoidObjects) {
    auto command = PositionControlCommand{};
    if (collisionDetector.isOccupied(targetPosition, robotId, avoidObjects)) {
        command.isOccupied = true;
        return command;
    }

    if (!pathControllers.contains(robotId)) {
        pathControllers.insert({robotId, {BBTPathPlanning(field.getFieldWidth(), maxRobotVelocity, robotId, collisionDetector), BBTPathTracking(robotId, collisionDetector)}});
    }

    auto &[pathPlanning, pathTracking] = pathControllers.at(robotId);
    auto shouldUpdate = pathTracking.shouldUpdatePath(currentPosition, targetPosition, avoidObjects);
    if (shouldUpdate != DONT_UPDATE) {
        pathPlanning.updateConstraints(field, avoidObjects, maxRobotVelocity);
        auto path = pathPlanning.generateNewPath(currentPosition, currentVelocity, targetPosition);
        interface::Input::drawData(interface::Visual::DEBUG, path, Qt::magenta, robotId, interface::Drawing::CROSSES);
        pathTracking.updatePath(std::move(path));
    }

    auto trackingVelocity = pathTracking.trackPathForwardAngle(currentPosition, currentVelocity, pidType);
    collisionDetector.updateTimelineForOurRobot(pathTracking.getRemainingPath(), currentPosition, robotId);

    command.robotCommand.velocity = {trackingVelocity.x, trackingVelocity.y};
    command.robotCommand.targetAngle = trackingVelocity.rot;
    return command;
}

CollisionDetector &PositionControl::getCollisionDetector() { return collisionDetector; }

}  // namespace rtt::ai::control