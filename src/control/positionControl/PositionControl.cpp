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

        if (shouldRecalculatePath(currentPosition, targetPosition, currentVelocity, robotId)) {
            computedPaths[robotId] = pathPlanningAlgorithm.computePath(currentPosition, targetPosition);
        }



        //Draw current path planning points
        interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::green, robotId,
                                   interface::Drawing::LINES_CONNECTED);
        interface::Input::drawData(interface::Visual::PATHFINDING, {computedPaths[robotId].front(), currentPosition},
                                   Qt::green, robotId, interface::Drawing::LINES_CONNECTED);
        interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::blue, robotId,
                                   interface::Drawing::DOTS);


        this->computeBBTPath(currentPosition, currentVelocity, targetPosition, robotId);

        RobotCommand command = RobotCommand();
        command.pos = computedPaths[robotId].front();
        Position trackingVelocity = pathTrackingAlgorithm.trackPathDefaultAngle(currentPosition, currentVelocity,
                                                                                computedPaths[robotId], robotId,
                                                                                pidType);
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

    BB::BBTrajectory2D
    PositionControl::computeBBTPath(Vector2 currentPosition, Vector2 currentVelocity, Vector2 targetPosition,
                                    int robotId) {
        double timeStep = 0.1;

        std::optional<BB::CollisionData> firstCollision;
        BB::BBTrajectory2D BBTPath;

        BBTPath = BB::BBTrajectory2D(currentPosition, currentVelocity, targetPosition,
                                         ai::Constants::MAX_VEL(), ai::Constants::MAX_ACC_UPPER());

        interface::Input::drawData(interface::Visual::PATHFINDING, BBTPath.getPathApproach(timeStep), Qt::magenta,
                                   robotId,interface::Drawing::DOTS);

        firstCollision = worldObjects.getFirstCollision(BBTPath, robotId);

        if (!firstCollision.has_value()) {
            return BBTPath;
        } else {
            double angleBetweenIntermediatePoints = M_PI_4 / 2;
            std::vector<Vector2> greenCrosses;

            Vector2 pointToDrawFrom = firstCollision->obstaclePosition +
                                      (firstCollision->obstaclePosition - targetPosition).normalize() * 1;

            for (int i = -4; i < 5; i++) {
                if (i != 0) {
                    greenCrosses.emplace_back(
                            firstCollision->obstaclePosition.rotateAroundPoint(i * angleBetweenIntermediatePoints,
                                                                                pointToDrawFrom));
                }
            }
            interface::Input::drawData(interface::Visual::PATHFINDING, greenCrosses, Qt::green, robotId,
                                       interface::Drawing::CROSSES);

            double greenCrossScore;
            std::priority_queue<std::pair<double, Vector2>, std::vector<std::pair<double, Vector2>>, std::greater<>> greenCrossesSorted;
            BB::BBTrajectory2D pathToIntermediatePoint;
            BB::BBTrajectory2D intermediateToTarget;
            for (auto i : greenCrosses) {
                std::vector<Vector2> collisionPoint = {firstCollision->collisionPosition};
                interface::Input::drawData(interface::Visual::PATHFINDING, collisionPoint,
                                           Qt::red, robotId,interface::Drawing::CROSSES);
                greenCrossScore = (i - firstCollision->collisionPosition).length();

                std::pair<double, Vector2> p = {greenCrossScore, i};

                greenCrossesSorted.push(p);
            }
            while (!greenCrossesSorted.empty()) {
                std::cout << "Score of greenCross: " << greenCrossesSorted.top().first << std::endl;
                pathToIntermediatePoint = BB::BBTrajectory2D(currentPosition, currentVelocity, greenCrossesSorted.top().second,
                                                    ai::Constants::MAX_VEL(), ai::Constants::MAX_ACC_UPPER());
                auto intermediatePathCollision = worldObjects.getFirstCollision(pathToIntermediatePoint, robotId);

                if (!intermediatePathCollision.has_value()) {
                    for (int i = 0; i < floor(pathToIntermediatePoint.getTotalTime() / (timeStep * 2)); i++) {
                        //TODO: Only create newStart's up to point where we dont decelerate yet
                        Vector2 newStart = pathToIntermediatePoint.getPosition(i * 2 * timeStep);
                        Vector2 newVelocity = pathToIntermediatePoint.getVelocity(i * 2 * timeStep);

                        intermediateToTarget = BB::BBTrajectory2D(newStart, newVelocity, targetPosition,
                                                                     ai::Constants::MAX_VEL(),
                                                                     ai::Constants::MAX_ACC_UPPER());

                        auto newStartCollisions = worldObjects.getFirstCollision(intermediateToTarget, robotId);

                        if (newStartCollisions.has_value()) {
                            continue;
                        } else {
                            greenCrosses.clear();
                            while(!greenCrossesSorted.empty()) {
                                greenCrossesSorted.pop();
                            }
                            return intermediateToTarget;
                        }
                    }
                }
                greenCrossesSorted.pop();
            }
            //TODO: White and yellow lines used to be drawn more often and at the same time, now they are not drawn at all
            //TODO: Find out why, maybe move the draw lines below to different places see if that fixes it?
            interface::Input::drawData(interface::Visual::PATHFINDING, pathToIntermediatePoint.getPathApproach(timeStep),
                                       Qt::white, robotId,
                                       interface::Drawing::LINES_CONNECTED);
            interface::Input::drawData(interface::Visual::PATHFINDING,
                                       intermediateToTarget.getPathApproach(timeStep),
                                       Qt::yellow, robotId, interface::Drawing::LINES_CONNECTED);
            return BBTPath;
        }
    }
}  // namespace rtt::ai// ::control