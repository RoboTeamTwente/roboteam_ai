//
// Created by ratoone on 18-11-19.
//

#include "control/positionControl/PositionControl.h"

#include "roboteam_utils/Print.h"
#include "stp/StpInfo.h"
#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"

namespace rtt::ai::control {
    RobotCommand
    PositionControl::computeAndTrackPath(const rtt::world::Field &field, int robotId, const Vector2 &currentPosition,
                                         const Vector2 &currentVelocity,
                                         const Vector2 &targetPosition, stp::PIDType pidType) {
        collisionDetector.setField(field);
        worldObjects.setField(field);
        // if the target position is outside of the field (i.e. bug in AI), do nothing
        if (!collisionDetector.isPointInsideField(targetPosition)) {
            RTT_WARNING("Target point not in field for robot ID ", robotId)
            return {};
        }

        // if the robot is close to the final position and can't get there, stop
        if ((currentPosition - targetPosition).length() < FINAL_AVOIDANCE_DISTANCE &&
            collisionDetector.getRobotCollisionBetweenPoints(currentPosition, targetPosition)) {
            RTT_INFO("Path collides with something close to the target position for robot ID ", robotId)
            return {};
        }

        //TODO: let the robot follow the BBT
        //currently the robot remains driving in the same direction as at the start of the BBT
        //this is caused by how the data is send to control
        if(!computedPaths.contains(robotId) ||
        (targetPosition - computedPaths[robotId].getPosition(computedPaths[robotId].getTotalTime())).length() > 0.01 ||
        worldObjects.getFirstCollision(computedPaths[robotId],robotId).has_value()){
            computedPaths[robotId] = this->computePath(field, currentPosition, currentVelocity, targetPosition, robotId);
        }

        auto pathApproach = computedPaths[robotId].getPathApproach(0.1);

        //Draw current path planning points
//        interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::green, robotId,
//                                   interface::Drawing::LINES_CONNECTED);
//        interface::Input::drawData(interface::Visual::PATHFINDING, {computedPaths[robotId].front(), currentPosition},
//                                   Qt::green, robotId, interface::Drawing::LINES_CONNECTED);
        interface::Input::drawData(interface::Visual::PATHFINDING, pathApproach, Qt::blue, robotId,
                                   interface::Drawing::DOTS);



        RobotCommand command = RobotCommand();
        command.pos = pathApproach.front();
        Position trackingVelocity = pathTrackingAlgorithm.trackPathDefaultAngle(currentPosition, currentVelocity,
                                                                                pathApproach, robotId,
                                                                                pidType);
        command.vel = computedPaths[robotId].getVelocity(0.1);
        command.angle = trackingVelocity.rot;

        return command;
    }

//    bool PositionControl::shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos,
//                                                const Vector2 &currentVelocity, int robotId) {
//        return computedPaths[robotId].empty() ||
//               PositionControlUtils::isTargetChanged(targetPos, computedPaths[robotId].back()) ||
//               (currentVelocity != Vector2(0, 0) &&
//                collisionDetector.isCollisionBetweenPoints(currentPosition, computedPaths[robotId].front()));
//    }

    void PositionControl::setRobotPositions(std::vector<Vector2> &robotPositions) {
        collisionDetector.setRobotPositions(robotPositions);
    }

    BB::BBTrajectory2D
    PositionControl::computePath(const rtt::world::Field &field, Vector2 currentPosition, Vector2 currentVelocity,
                                 Vector2 targetPosition, int robotId) {
        //TODO: find a good value for the timeStep
        double timeStep = 0.1;

        //Create path to original target
        BB::BBTrajectory2D originalPath = BB::BBTrajectory2D(currentPosition, currentVelocity, targetPosition,
                                                             ai::Constants::MAX_VEL(), ai::Constants::MAX_ACC_UPPER());

        interface::Input::drawData(interface::Visual::PATHFINDING, originalPath.getPathApproach(0.3),
                                   Qt::magenta, robotId, interface::Drawing::DOTS);

        //Check path to original target for collisions
        std::optional<BB::CollisionData> firstCollision = worldObjects.getFirstCollision(originalPath, robotId);

        if (firstCollision.has_value()) {
            //Create intermediate points, return a collision-free path originating from the best option of these points
            auto newPath = findNewPath(currentPosition, currentVelocity, firstCollision, targetPosition, field, robotId,
                                       timeStep);
            if (newPath.has_value()) {
                return newPath.value();
            }
        }
        return originalPath;
    }

    std::optional<BB::BBTrajectory2D>
    PositionControl::findNewPath(Vector2 &currentPosition, Vector2 &currentVelocity,
                                 std::optional<BB::CollisionData> &firstCollision,
                                 Vector2 &targetPosition, const rtt::world::Field &field, int robotId,
                                 double timeStep) {

        auto intermediatePoints = createIntermediatePoints(firstCollision, targetPosition, field, robotId);
        auto intermediatePointsSorted = sortIntermediatePoints(intermediatePoints, firstCollision);

        BB::BBTrajectory2D pathToIntermediatePoint;
        while (!intermediatePointsSorted.empty()) {
            //TODO: Make sure that when a robot drives towards this intermediate point, it doesnt reach vel = 0.
            // So maybe instead of using the intermediatePoint as its target, use the last two points of the path towards
            // the point, calculate a drivingDirection, and extend the target beyond the point in this direction.
            pathToIntermediatePoint = BB::BBTrajectory2D(currentPosition, currentVelocity,
                                                         intermediatePointsSorted.top().second,
                                                         ai::Constants::MAX_VEL(), ai::Constants::MAX_ACC_UPPER());

            auto intermediatePathCollision = worldObjects.getFirstCollision(pathToIntermediatePoint, robotId);
            auto intermediateToTarget = calculatePathFromNewStart(intermediatePathCollision, pathToIntermediatePoint,
                                                                  targetPosition, robotId, timeStep);
            if (intermediateToTarget.has_value()) {
                interface::Input::drawData(interface::Visual::PATHFINDING, intermediatePoints, Qt::green, robotId,
                                           interface::Drawing::CROSSES);
                interface::Input::drawData(interface::Visual::PATHFINDING, {firstCollision->collisionPosition},
                                           Qt::red, robotId, interface::Drawing::CROSSES);

                interface::Input::drawData(interface::Visual::PATHFINDING,
                                           pathToIntermediatePoint.getPathApproach(timeStep),
                                           Qt::white, robotId,
                                           interface::Drawing::LINES_CONNECTED);
                interface::Input::drawData(interface::Visual::PATHFINDING,
                                           intermediateToTarget.value().getPathApproach(timeStep),
                                           Qt::yellow, robotId, interface::Drawing::LINES_CONNECTED);
                return intermediateToTarget.value();
            }
            intermediatePointsSorted.pop();
        }
        return std::nullopt;
    }

    std::vector<Vector2>
    PositionControl::createIntermediatePoints(std::optional<BB::CollisionData> &firstCollision, Vector2 &targetPosition,
                                              const rtt::world::Field &field, int robotId) {
        double angleBetweenIntermediatePoints = M_PI_4 / 2;

        Vector2 pointToDrawFrom = firstCollision->obstaclePosition +
                                  (firstCollision->obstaclePosition - targetPosition).normalize() * 1;

        std::vector<Vector2> intermediatePoints;
        for (int i = -4; i < 5; i++) {
            if (i != 0) {
                //Make half circle of intermediatePoints originating from position of obstacle that caused collision
                Vector2 intermediatePoint = firstCollision->obstaclePosition.rotateAroundPoint(
                        i * angleBetweenIntermediatePoints,
                        pointToDrawFrom);

                //If not in a defense area (only checked if robot is not allowed in defense area)
                if (worldObjects.canEnterDefenseArea(robotId) ||
                    (!rtt::ai::FieldComputations::pointIsInDefenseArea(field, intermediatePoint, true, 0) &&
                     !rtt::ai::FieldComputations::pointIsInDefenseArea(field, intermediatePoint, false,
                                                                       0.2 + rtt::ai::Constants::ROBOT_RADIUS()))) {
                    //.. and inside the field (only checked if the robot is not allowed outside the field), add this cross to the list
                    if (worldObjects.canMoveOutsideField(robotId) ||
                        rtt::ai::FieldComputations::pointIsInField(field, intermediatePoint,
                                                                   rtt::ai::Constants::ROBOT_RADIUS())) {
                        intermediatePoints.emplace_back(intermediatePoint);
                    }
                }
            }
        }
        return intermediatePoints;
    }

    std::priority_queue<std::pair<double, Vector2>, std::vector<std::pair<double, Vector2>>, std::greater<>>
    PositionControl::sortIntermediatePoints(std::vector<Vector2> &intermediatePoints,
                                            std::optional<BB::CollisionData> &firstCollision) {
        double intermediatePointScore;
        std::priority_queue<std::pair<double, Vector2>, std::vector<std::pair<double, Vector2>>, std::greater<>> intermediatePointsSorted;
        for (auto i : intermediatePoints) {
            intermediatePointScore = (i - firstCollision->collisionPosition).length();
            std::pair<double, Vector2> p = {intermediatePointScore, i};
            intermediatePointsSorted.push(p);
        }
        return intermediatePointsSorted;
    }

    std::optional<BB::BBTrajectory2D>
    PositionControl::calculatePathFromNewStart(std::optional<BB::CollisionData> intermediatePathCollision,
                                               BB::BBTrajectory2D pathToIntermediatePoint,
                                               Vector2 &targetPosition, int robotId, double timeStep) {
        BB::BBTrajectory2D intermediateToTarget;
        if (!intermediatePathCollision.has_value()) {
            timeStep *= 2;
            for (int i = 0; i < floor(pathToIntermediatePoint.getTotalTime() / timeStep); i++) {
                //TODO: Only create newStart's up to point where we dont decelerate yet
                Vector2 newStart = pathToIntermediatePoint.getPosition(i * timeStep);
                Vector2 newVelocity = pathToIntermediatePoint.getVelocity(i * timeStep);

                intermediateToTarget = BB::BBTrajectory2D(newStart, newVelocity, targetPosition,
                                                          ai::Constants::MAX_VEL(),
                                                          ai::Constants::MAX_ACC_UPPER());
                auto newStartCollisions = worldObjects.getFirstCollision(intermediateToTarget, robotId);

                if (newStartCollisions.has_value()) {
                    continue;
                } else {
                    return intermediateToTarget;
                }
            }
        }
        return std::nullopt;
    }
}  // namespace rtt::ai// ::control