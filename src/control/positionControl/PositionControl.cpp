//
// Created by ratoone on 18-11-19.
//

#include "control/positionControl/PositionControl.h"

#include "roboteam_utils/Print.h"
#include "stp/StpInfo.h"
#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"

namespace rtt::ai::control {
RobotCommand PositionControl::computeAndTrackPath(const rtt::world::Field &field, int robotId, const Vector2 &currentPosition, const Vector2 &currentVelocity,
                                                  const Vector2 &targetPosition, stp::PIDType pidType) {
    collisionDetector.setField(field);
    worldObjects.setField(field);
    // if the target position is outside of the field (i.e. bug in AI), do nothing
    if (!collisionDetector.isPointInsideField(targetPosition)) {
        RTT_WARNING("Target point not in field for robot ID ", robotId)
        return {};
    }

    // if the robot is close to the final position and can't get there, stop
    if ((currentPosition - targetPosition).length() < FINAL_AVOIDANCE_DISTANCE && collisionDetector.getRobotCollisionBetweenPoints(currentPosition, targetPosition)) {
        RTT_INFO("Path collides with something close to the target position for robot ID ", robotId)
        return {};
    }

    if (shouldRecalculatePath(currentPosition, targetPosition, currentVelocity, robotId)) {
        computedPaths[robotId] = pathPlanningAlgorithm.computePath(currentPosition, targetPosition);
    }

    BB::BBTrajectory2D test = BB::BBTrajectory2D(currentPosition,currentVelocity,targetPosition,ai::Constants::MAX_VEL(),ai::Constants::MAX_ACC_UPPER());
    std::vector<Vector2> points;
    std::vector<Vector2>* pointsPtr = &points;
    points = test.getPathApproach(0.1);
    std::vector<Vector2> collisions;
    do {
        //worldObjects.getPath();
        collisions = worldObjects.collisionChecker(test, robotId);
        if(!collisions.empty()) {
            std::cout << "My soul is not empty babyyy" << std::endl;
            //BB::BBTrajectory2D newPath = test.getPath(collisions);
        }
        worldObjects.storeCalculatedPath(pointsPtr,robotId);
    } while (!collisions.empty());

    interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::green, robotId, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::PATHFINDING, {computedPaths[robotId].front(), currentPosition}, Qt::green, robotId, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::blue, robotId, interface::Drawing::DOTS);

    interface::Input::drawData(interface::Visual::PATHFINDING,points,Qt::white,robotId,interface::Drawing::DOTS);
    if(!collisions.empty()) {
        interface::Input::drawData(interface::Visual::PATHFINDING, collisions, Qt::red, robotId,
                                   interface::Drawing::CROSSES);
    }

    RobotCommand command = RobotCommand();
    command.pos = computedPaths[robotId].front();
    Position trackingVelocity = pathTrackingAlgorithm.trackPathDefaultAngle(currentPosition, currentVelocity, computedPaths[robotId], robotId, pidType);
    command.vel = Vector2(trackingVelocity.x, trackingVelocity.y);
    command.angle = trackingVelocity.rot;

    return command;
}

bool PositionControl::shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos, const Vector2 &currentVelocity, int robotId) {
    return computedPaths[robotId].empty() || PositionControlUtils::isTargetChanged(targetPos, computedPaths[robotId].back()) ||
           (currentVelocity != Vector2(0, 0) && collisionDetector.isCollisionBetweenPoints(currentPosition, computedPaths[robotId].front()));
}

void PositionControl::setRobotPositions(std::vector<Vector2> &robotPositions) { collisionDetector.setRobotPositions(robotPositions); }

}  // namespace rtt::ai// ::control

