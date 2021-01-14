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

        BB::BBTrajectory2D test = BB::BBTrajectory2D(currentPosition, currentVelocity, targetPosition,
                                                     ai::Constants::MAX_VEL(), ai::Constants::MAX_ACC_UPPER());
        std::vector<Vector2> points;
        std::vector<Vector2> *pointsPtr = &points;
        points = test.getPathApproach(0.1);
        Vector2 firstCollision = {20, 20};
        Vector2 drivingDirection;
        std::vector<Vector2> vectorsToDraw;
        do {
            //std::vector<Vector2> temp = worldObjects.getFirstCollision(test, robotId);
            firstCollision = {0,2};
            //drivingDirection = {1,0};
            if (firstCollision.x != 20 && firstCollision.y != 20) {
                std::cout << "My soul is not empty babyyy" << std::endl;
                Vector2 leftIntermediatePos = (firstCollision +
                                               Vector2(drivingDirection.y, -drivingDirection.x).stretchToLength(
                                                       4*FINAL_AVOIDANCE_DISTANCE)).rotateAroundPoint(5*(M_PI_4/2),
                                                                                                    firstCollision);
                Vector2 rightIntermediatePos = (firstCollision +
                                                Vector2(drivingDirection.y, -drivingDirection.x).stretchToLength(
                                                        -4*FINAL_AVOIDANCE_DISTANCE)).rotateAroundPoint(-5*(M_PI_4/2),
                                                                                                      firstCollision);
                vectorsToDraw = {firstCollision, leftIntermediatePos, rightIntermediatePos};

                //BB::BBTrajectory2D newPath = getNewPath(test, firstCollision);

                //1. Maak 2 intermediate points perpendicular op rijrichting, vanuit obstacle waar de collision door komt.                      ALMOST DONE
                //2. Kies het punt wat het dichtsbij het huidige pad zit, maak een pad en bereken daarvoor opnieuw collisions uit.              SOON TM
                //3. Als daar collisions in zitten, voer vanuit dit nieuwe pad stap 1 en 2 opnieuw uit, totdat er een pad is zonder collisions. SOON TM
            }
            worldObjects.storeCalculatedPath(pointsPtr, robotId);
        } while (firstCollision.x == 20 && firstCollision.y == 20);

        //Draw current path planning points
        interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::green, robotId,
                                   interface::Drawing::LINES_CONNECTED);
        interface::Input::drawData(interface::Visual::PATHFINDING, {computedPaths[robotId].front(), currentPosition},
                                   Qt::green, robotId, interface::Drawing::LINES_CONNECTED);
        interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::blue, robotId,
                                   interface::Drawing::DOTS);

        //Draw all BB pathPoints
        interface::Input::drawData(interface::Visual::PATHFINDING, points, Qt::white, robotId,
                                   interface::Drawing::DOTS);

        //Draw collisions and intermediate points
        if (firstCollision.x != 20 && firstCollision.y != 20) {
            interface::Input::drawData(interface::Visual::PATHFINDING, vectorsToDraw, Qt::red, robotId,
                                       interface::Drawing::CROSSES);
        }

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

    BB::BBTrajectory2D PositionControl::getNewPath(BB::BBTrajectory2D currentPath, Vector2 &collisions) {


        return BB::BBTrajectory2D();
    }

}  // namespace rtt::ai// ::control

