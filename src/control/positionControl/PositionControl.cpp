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

    BB::BBTrajectory2D
    PositionControl::computeBBTPath(Vector2 currentPosition, Vector2 currentVelocity, Vector2 targetPosition,
                                    int robotId, bool originalPath) {
        double timeStep = 0.1;

        std::optional<BB::CollisionData> firstCollision;
        //std::vector<Vector2> vectorsToDraw;
        std::optional<BB::BBTrajectory2D> newPath;
        BB::BBTrajectory2D BBTPath;
        std::vector<Vector2> points;
        if(originalPath) {
            //TODO: make sure this is executed once and after that recursively do the while loop below
            BBTPath = BB::BBTrajectory2D(currentPosition, currentVelocity, targetPosition,
                                                            ai::Constants::MAX_VEL(), ai::Constants::MAX_ACC_UPPER());
            points = BBTPath.getPathApproach(timeStep);
        }
            do {
                firstCollision = worldObjects.getFirstCollision(BBTPath, robotId);
                if (firstCollision.has_value()) {
                    std::cout << "My soul is not empty babyyy" << std::endl;
                    Vector2 leftIntermediatePos = (firstCollision->collisionPosition +
                                                   Vector2(firstCollision->drivingDirection.y,
                                                           firstCollision->drivingDirection.x * -1).stretchToLength(
                                                           4 * FINAL_AVOIDANCE_DISTANCE)).rotateAroundPoint(
                            M_PI_4 / 2, firstCollision->collisionPosition);
                    Vector2 rightIntermediatePos = (firstCollision->collisionPosition +
                                                    Vector2(firstCollision->drivingDirection.y,
                                                            firstCollision->drivingDirection.x * -1).stretchToLength(
                                                            -4 * FINAL_AVOIDANCE_DISTANCE)).rotateAroundPoint(
                            -1 * M_PI_4 / 2, firstCollision->collisionPosition);
                    //vectorsToDraw = {firstCollision, leftIntermediatePos, rightIntermediatePos};

                    newPath = getNewPath(currentPosition, currentVelocity, leftIntermediatePos, rightIntermediatePos);

                    BBTPath = newPath.value();
                }
                worldObjects.storeCalculatedPath(BBTPath.getPathApproach(timeStep), robotId);
            } while (firstCollision.has_value());

        //1. Kies het punt wat het dichtsbij het huidige pad zit, maak een pad en bereken daarvoor opnieuw collisions uit.              SOON TM
        //2. Als daar collisions in zitten, voer vanuit dit nieuwe pad stap 1 en 2 opnieuw uit, totdat er een pad is zonder collisions. SOON TM
        return BBTPath;
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

