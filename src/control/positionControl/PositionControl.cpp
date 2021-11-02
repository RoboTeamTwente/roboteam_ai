//
// Created by ratoone on 18-11-19.
//

#include "control/positionControl/PositionControl.h"

#include "roboteam_utils/Print.h"
#include "stp/StpInfo.h"
#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"

namespace rtt::ai::control {
    RobotCommand
    PositionControl::computeAndTrackPath(const rtt::world::Field &field, int robotId, const Vector2 &currentPosition, const Vector2 &currentVelocity, Vector2 &targetPosition,
                                     stp::PIDType pidType) {
        collisionDetector.setField(field);

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
        if (shouldRecalculatePath(currentPosition, targetPosition, currentVelocity, robotId)) {
            computedPaths[robotId] = pathPlanningAlgorithm.computePath(currentPosition, targetPosition);
        }
        interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::green, robotId, interface::Drawing::LINES_CONNECTED);
        interface::Input::drawData(interface::Visual::PATHFINDING, {computedPaths[robotId].front(), currentPosition}, Qt::green, robotId, interface::Drawing::LINES_CONNECTED);
        interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::blue, robotId, interface::Drawing::DOTS);


        RobotCommand command = RobotCommand();
        command.pos = computedPaths[robotId].front();
        Position trackingVelocity = pathTrackingAlgorithm.trackPathDefaultAngle(currentPosition, currentVelocity,computedPaths[robotId], robotId,pidType);
        command.vel = Vector2(trackingVelocity.x, trackingVelocity.y);
        command.angle = trackingVelocity.rot;
        return command;
    }

    bool PositionControl::shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos,
                                                const Vector2 &currentVelocity, int robotId) {
        return computedPaths[robotId].empty() ||
               PositionControlUtils::isTargetChanged(targetPos, computedPaths[robotId].back()) ||
               (currentVelocity != Vector2(0, 0) &&
                collisionDetector.isCollisionBetweenPoints(currentPosition, computedPaths[robotId].front()));
    }

    void PositionControl::setRobotPositions(std::vector<Vector2> &robotPositions) {
        collisionDetector.setRobotPositions(robotPositions);
    }

    rtt::BB::CommandCollision PositionControl::computeAndTrackPathBBT(const rtt::world::World *world, const rtt::world::Field &field, int robotId, Vector2 currentPosition,
                                                                      Vector2 currentVelocity,Vector2 targetPosition, stp::PIDType pidType) {
        //TODO: split this function in a compute and a track part
        //TODO: find a good value for the timeStep
        double timeStep = 0.1;

        std::optional<BB::CollisionData> firstCollision;
        rtt::BB::CommandCollision commandCollision;
        // Currently calculate all paths again on each tick because the way the path is used in control is not made for the BBT
        // When the path tracking is fixed the true in the if statement can be removed such that it only calculates the path again when it needs to
        if (false || (!computedPathsBB.contains(robotId) ||
            (targetPosition - computedPathsBB[robotId].getPosition(computedPathsBB[robotId].getTotalTime())).length() > stp::control_constants::GO_TO_POS_ERROR_MARGIN ||
            worldObjects.getFirstCollision(world, field, computedPathsBB[robotId], computedPaths, robotId).has_value())) {
            //Create path to original target
            computedPathsBB[robotId] = BB::BBTrajectory2D(currentPosition, currentVelocity, targetPosition,ai::Constants::MAX_VEL(), ai::Constants::MAX_ACC_UPPER());

            //Check path to original target for collisions
            firstCollision = worldObjects.getFirstCollision(world, field, computedPathsBB[robotId], computedPaths, robotId);

            if (firstCollision.has_value()) {
                //Create intermediate points, return a collision-free path originating from the best option of these points
                auto newPath = findNewPath(world, field, robotId, currentPosition, currentVelocity, firstCollision, targetPosition,timeStep);
                if (newPath.has_value()) {
                    computedPathsBB[robotId] = newPath.value();
                } else {
                    commandCollision.collisionData = firstCollision;
                }
            }
            computedPaths[robotId] = computedPathsBB[robotId].getPathApproach(0.2);
        }
        interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::yellow, robotId, interface::Drawing::LINES_CONNECTED);
        interface::Input::drawData(interface::Visual::PATHFINDING, {computedPaths[robotId].front(), currentPosition}, Qt::darkMagenta, robotId, interface::Drawing::LINES_CONNECTED);
        interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::magenta, robotId, interface::Drawing::DOTS);

        // TODO: let the robot properly follow the BBT
        //Current method is very hacky
        // If you are closer to the target than the first point of the approximated path, remove it
        if((targetPosition - currentPosition).length() < (targetPosition-computedPaths[robotId].front()).length()){
            computedPaths[robotId].erase(computedPaths[robotId].begin());
        }

        commandCollision.robotCommand = RobotCommand();
        commandCollision.robotCommand.pos = computedPaths[robotId].front();
        Position trackingVelocity = pathTrackingAlgorithm.trackPathDefaultAngle(currentPosition, currentVelocity,computedPaths[robotId], robotId,pidType);
        commandCollision.robotCommand.vel = Vector2(trackingVelocity.x, trackingVelocity.y);
        commandCollision.robotCommand.angle = trackingVelocity.rot;

        return commandCollision;
    }

    std::optional<BB::BBTrajectory2D>
    PositionControl::findNewPath(const rtt::world::World *world, const rtt::world::Field &field, int robotId, Vector2 &currentPosition, Vector2 &currentVelocity,
                                 std::optional<BB::CollisionData> &firstCollision, Vector2 &targetPosition, double timeStep) {

        auto intermediatePoints = createIntermediatePoints(field, robotId, firstCollision, targetPosition);
        auto intermediatePointsSorted = scoreIntermediatePoints(intermediatePoints, firstCollision);

        BB::BBTrajectory2D pathToIntermediatePoint;
        while (!intermediatePointsSorted.empty()) {
            //TODO: Make sure that when a robot drives towards this intermediate point, it doesnt reach vel = 0.
            // So maybe instead of using the intermediatePoint as its target, use the last two points of the path towards
            // the point, calculate a drivingDirection, and extend the target beyond the point in this direction.
            pathToIntermediatePoint = BB::BBTrajectory2D(currentPosition, currentVelocity,
                                                         intermediatePointsSorted.top().second,
                                                         ai::Constants::MAX_VEL(), ai::Constants::MAX_ACC_UPPER());

            auto intermediatePathCollision = worldObjects.getFirstCollision(world, field, pathToIntermediatePoint, computedPaths, robotId);
            auto intermediateToTarget = calculatePathFromNewStart(world, field, intermediatePathCollision, pathToIntermediatePoint,
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
    PositionControl::createIntermediatePoints(const rtt::world::Field &field, int robotId, std::optional<BB::CollisionData> &firstCollision,
                                              Vector2 &targetPosition) {
        double angleBetweenIntermediatePoints = M_PI_4 / 2;

        // PointToDrawFrom is picked by drawing a line from the target position to the obstacle and extending that
        // line further towards our currentPosition by extension meters.
        float pointExtension = 0.5;  // How far the pointToDrawFrom has to be from the obstaclePosition
        Vector2 pointToDrawFrom = firstCollision->obstaclePosition +
                                  (firstCollision->obstaclePosition - targetPosition).normalize() * pointExtension;

        std::vector<Vector2> intermediatePoints;
        for (int i = -4; i < 5; i++) {
            if (i != 0) {
                //Make half circle of intermediatePoints pointed towards obstaclePosition, originating from pointToDrawFrom, by rotating pointToRotate with a radius intermediatePointRadius
                float intermediatePointRadius = 2; // Radius of the half circle
                Vector2 pointToRotate = pointToDrawFrom + (targetPosition - firstCollision->obstaclePosition).normalize() * intermediatePointRadius;
                Vector2 intermediatePoint = pointToRotate.rotateAroundPoint(i * angleBetweenIntermediatePoints, pointToDrawFrom);

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
    PositionControl::scoreIntermediatePoints(std::vector<Vector2> &intermediatePoints,
                                            std::optional<BB::CollisionData> &firstCollision) {
        double intermediatePointScore;
        std::priority_queue<std::pair<double, Vector2>, std::vector<std::pair<double, Vector2>>, std::greater<>> intermediatePointsSorted;
        for (const auto& i : intermediatePoints) {
            intermediatePointScore = (i - firstCollision->collisionPosition).length();
            std::pair<double, Vector2> p = {intermediatePointScore, i};
            intermediatePointsSorted.push(p);
        }
        return intermediatePointsSorted;
    }

    std::optional<BB::BBTrajectory2D>
    PositionControl::calculatePathFromNewStart(const rtt::world::World *world, const rtt::world::Field &field, std::optional<BB::CollisionData> intermediatePathCollision,
                                               BB::BBTrajectory2D pathToIntermediatePoint,
                                               Vector2 &targetPosition, int robotId, double timeStep) {
        BB::BBTrajectory2D intermediateToTarget;
        if (!intermediatePathCollision.has_value()) {
            timeStep *= 2;
            int numberOfTimeSteps = floor(pathToIntermediatePoint.getTotalTime() / timeStep);
            for (int i = 0; i < numberOfTimeSteps; i++) {
                //TODO: Only create newStart's up to point where we dont decelerate yet
                Vector2 newStart = pathToIntermediatePoint.getPosition(i * timeStep);
                Vector2 newVelocity = pathToIntermediatePoint.getVelocity(i * timeStep);

                intermediateToTarget = BB::BBTrajectory2D(newStart, newVelocity, targetPosition,
                                                          ai::Constants::MAX_VEL(),
                                                          ai::Constants::MAX_ACC_UPPER());
                auto newStartCollisions = worldObjects.getFirstCollision(world, field, intermediateToTarget, computedPaths, robotId);

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