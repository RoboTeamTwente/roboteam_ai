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


        this->computeBBTPath(currentPosition, Vector2(2, 2), targetPosition, 1, true);
        /*
        //Draw all BB pathPoints
        interface::Input::drawData(interface::Visual::PATHFINDING, points, Qt::white, robotId,
                                   interface::Drawing::DOTS);
        interface::Input::drawData(interface::Visual::PATHFINDING, newPathPoints, Qt::white, robotId,
                                   interface::Drawing::DOTS);

        //Draw collisions and intermediate points
        if (firstCollision.x != 20 && firstCollision.y != 20) {
            interface::Input::drawData(interface::Visual::PATHFINDING, vectorsToDraw, Qt::red, robotId,
                                       interface::Drawing::CROSSES);
        }
         */

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


    //TODO: Use pseudocode below, for creating non-recursive way of finding best path without collisions
//    using std::priority_queue<PointsToCalculate> pqueue;
//
//    BB::BBTrajectory2D computeBBPath(Vector2 currentPosition, Vector2 currentVelocity, Vector2 targetPosition, int robotId) {
//
//        double angleBetweenIntermediatePoints = M_PI_4/2;
//        Vector2 drivingDirection = Vector2(drivingDirection);
//
//        BBTPath = BB::BBTrajectory2D(currentPosition, currentVelocity, targetPosition,
//                                     ai::Constants::MAX_VEL(), ai::Constants::MAX_ACC_UPPER());
//        auto firstCollision = worldObjects.getFirstCollision(BBTPath, robotId);
//
//        if(noCollission) {
//            return path;
//        } else {
//            //createIntermediatePoints returns 8 points left and right of collision
//            pqueue.add(createIntermediatePoints(angleBetweenIntermediatePoints, drivingDirection));
//
//            Vector2 bestIntPoint = pqueue.top();
//
//            BBTrajectory2D intPath = generatePath(currentPos, currentVel, bestIntPoint);
//
//            //Only create redCrosses up to point where we dont decelerate yet
//            auto listOfRedCrosses = intPath.createRedCrosses(timeStep);
//            BBTrajectory2D listOfRedCrossPaths[listOfRedCrosses.size()];
//
//            for(auto i : listOfRedCrosses) {
//                listOfRedCrossPaths = generatePath(currentPos, currentVel, listOfRedCrosses[i]);
//            }
//
//            return listOfRedCrossPaths.selectBest();
//        }
//    }

    BB::BBTrajectory2D
    PositionControl::computeBBTPath(Vector2 currentPosition, Vector2 currentVelocity, Vector2 targetPosition,
                                    int robotId, bool originalPath) {
        double timeStep = 0.1;
        //targetPosition = Vector2{3,-3};

        std::optional<BB::CollisionData> firstCollision = BB::CollisionData{Vector2{0, 2.5}, Vector2{2, 0}};
        //std::vector<Vector2> vectorsToDraw;
        std::optional<BB::BBTrajectory2D> newPath;
        BB::BBTrajectory2D BBTPath;
        std::vector<Vector2> points;
        if (originalPath) {
            //TODO: make sure this is executed once and after that recursively do the while loop below
            BBTPath = BB::BBTrajectory2D(currentPosition, currentVelocity, targetPosition,
                                         ai::Constants::MAX_VEL(), ai::Constants::MAX_ACC_UPPER());
        }
//        do {
        //firstCollision = worldObjects.getFirstCollision(BBTPath, robotId);
        if (firstCollision.has_value()) {
            double angleBetweenIntermediatePoints = M_PI_4 / 2;

            Vector2 pointToDrawFrom = firstCollision->collisionPosition +
                                      (firstCollision->collisionPosition - targetPosition).normalize() * 1;

            std::vector<Vector2> greenCrosses;

            for (int i = -4; i < 5; i++) {
                if (i != 0) {
                    greenCrosses.emplace_back(
                            firstCollision->collisionPosition.rotateAroundPoint(i * angleBetweenIntermediatePoints,
                                                                                pointToDrawFrom));
                }
            }
            interface::Input::drawData(interface::Visual::PATHFINDING, greenCrosses, Qt::green, robotId,
                                       interface::Drawing::CROSSES);
            interface::Input::drawData(interface::Visual::PATHFINDING, BBTPath.getPathApproach(timeStep), Qt::magenta,
                                       robotId,
                                       interface::Drawing::DOTS);

            double greenCrossScore;
            std::priority_queue<std::pair<double, Vector2>, std::vector<std::pair<double, Vector2>>, std::greater<>> greenCrossesSorted;
            BB::BBTrajectory2D BBTPathcrosses;
            for (auto i : greenCrosses) {
                float targetWeight = 0.3;
                float positionWeight = 0.3;
                float velocityWeight = 0.4;

                auto angleDif = acos((i.dot(currentVelocity)) / (i.length() * currentVelocity.length()));

                greenCrossScore = ((i - targetPosition).length() / 12) * targetWeight +
                                  ((i - currentPosition).length() / 12) * positionWeight +
                                  (angleDif / M_PI) * velocityWeight;

                std::pair<double, Vector2> p = {greenCrossScore, i};

                greenCrossesSorted.push(p);

                BBTPathcrosses = BB::BBTrajectory2D(currentPosition, currentVelocity, i,
                                                    ai::Constants::MAX_VEL(), ai::Constants::MAX_ACC_UPPER());
                interface::Input::drawData(interface::Visual::PATHFINDING, BBTPathcrosses.getPathApproach(timeStep),
                                           Qt::white, robotId,
                                           interface::Drawing::LINES_CONNECTED);
            }
            while(!greenCrossesSorted.empty()) {
                std::cout << "Score of greenCross: " << greenCrossesSorted.top().first << std::endl;
                greenCrossesSorted.pop();
            }
        }
//                //vectorsToDraw = {firstCollision, leftIntermediatePos, rightIntermediatePos};
//
//                newPath = getNewPath(currentPosition, currentVelocity, leftIntermediatePos, rightIntermediatePos);
//
//                BBTPath = newPath.value();
//            }
//            worldObjects.storeCalculatedPath(BBTPath.getPathApproach(timeStep), robotId);
//        } while (firstCollision.has_value());
//
//        //1. Kies het punt wat het dichtsbij het huidige pad zit, maak een pad en bereken daarvoor opnieuw collisions uit.              SOON TM
//        //2. Als daar collisions in zitten, voer vanuit dit nieuwe pad stap 1 en 2 opnieuw uit, totdat er een pad is zonder collisions. SOON TM
//        return BBTPath;
    }

    BB::BBTrajectory2D
    PositionControl::getNewPath(const Vector2 &currentPosition, const Vector2 &currentVelocity,
                                Vector2 leftIntermediatePos, Vector2 rightIntermediatePos) {

        if ((currentVelocity - leftIntermediatePos).length() > (currentVelocity - rightIntermediatePos).length()) {
            return BB::BBTrajectory2D(currentPosition, currentVelocity, rightIntermediatePos, ai::Constants::MAX_VEL(),
                                      ai::Constants::MAX_ACC_UPPER());
        } else {
            return BB::BBTrajectory2D(currentPosition, currentVelocity, leftIntermediatePos, ai::Constants::MAX_VEL(),
                                      ai::Constants::MAX_ACC_UPPER());
        }
    }

}  // namespace rtt::ai// ::control