//
// Created by ratoone on 18-11-19.
//

#include "control/positionControl/PositionControl.h"

#include "roboteam_utils/Print.h"
#include "stp/StpInfo.h"
#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"

namespace rtt::ai::control {
    RobotCommand
    PositionControl::computeAndTrackPath(const rtt::world::Field &field, int robotId, const Vector2 &currentPosition, const Vector2 &currentVelocity, const Vector2 &targetPosition,
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

    rtt::BB::CommandCollision PositionControl::computePathBBT(const rtt::world::World *world, const rtt::world::Field &field, int robotId, Vector2 currentPosition,
                                                                      Vector2 currentVelocity, Vector2 targetPosition, std::optional<double> ballAvoidanceDistance, stp::PIDType pidType) {
        //TODO: find a good value for both timeSteps
        double pathTimeStep = 0.1;
        double velTimeStep = 1.0 / 60.0;

        std::optional<BB::CollisionData> firstCollision;
        rtt::BB::CommandCollision commandCollision;
        // Check if the path needs to be recalculated
        if (shouldRecalculateBBTPath(world, field, currentPosition, targetPosition, ballAvoidanceDistance, robotId, pathTimeStep, velTimeStep)) {
            //Create path to original target
            computedPathsBB[robotId].first = BB::BBTrajectory2D(currentPosition, currentVelocity, targetPosition,ai::Constants::MAX_VEL(), ai::Constants::MAX_ACC_UPPER());
            // Clear the data in the second part of the pair
            computedPathsBB[robotId].second = std::nullopt;
            computedPaths[robotId] = computedPathsBB[robotId].first.value().getPathApproach(pathTimeStep);
            computedVelocities[robotId] = computedPathsBB[robotId].first.value().getVelocityApproach(velTimeStep);

            //Check path to original target for collisions
            firstCollision = worldObjects.getFirstCollision(world, field, computedPaths, computedVelocities, ballAvoidanceDistance, robotId, pathTimeStep, velTimeStep);

            if (firstCollision.has_value()) {
                //Create intermediate points, return a collision-free path originating from the best option of these points
                std::pair<std::optional<BB::BBTrajectory2D>,std::optional<BB::BBTrajectory2D>> newPath =
                    findNewPath(world, field, robotId, currentPosition, currentVelocity, firstCollision, targetPosition, ballAvoidanceDistance, pathTimeStep, velTimeStep);

                if (newPath.first.has_value()) {
                    //If a new path is found update computedPathsBB
                    computedPathsBB[robotId] = newPath;
                    makeApproaches(robotId, pathTimeStep, velTimeStep);
                } else {
                    //Else update the collisionData. In GoToPos there is a check to see if collisionData is empty and will return Status Failure if not empty
                    commandCollision.collisionData = firstCollision;
                }
            }
        }
        interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::yellow, robotId, interface::Drawing::LINES_CONNECTED);
        interface::Input::drawData(interface::Visual::PATHFINDING, {computedPaths[robotId].front(), currentPosition}, Qt::darkMagenta, robotId, interface::Drawing::LINES_CONNECTED);
        interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::magenta, robotId, interface::Drawing::DOTS);

        // If you are closer to the target than the first point of the approximated path, remove it
        // TODO: Too many points are removed from computedPaths, fix this
        PositionControlUtils::removeFirstIfReached(computedPaths[robotId],currentPosition);

        commandCollision.robotCommand = RobotCommand();
        commandCollision.robotCommand.pos = computedPaths[robotId].front();

        return commandCollision;
    }

    bool PositionControl::shouldRecalculateBBTPath(const rtt::world::World *world, const rtt::world::Field &field,
                                                   const Vector2 &currentPosition, const Vector2 &targetPosition, std::optional<double> ballAvoidanceDistance, int robotId,
                                                   const double pathTimeStep, const double velTimeStep) {
        ai::GameState gameState = rtt::ai::GameStateManager::getCurrentGameState();
        ai::RuleSet ruleSet = gameState.getRuleSet();

        bool targetBool;

        //Check if there already has been a path calculated for the robot
        if(computedPathsBB.contains(robotId)) {
            //Check if the targetPosition differs by a certain margin from the end of the calculated path
            if (!computedPathsBB[robotId].second.has_value()) {
                targetBool = (targetPosition - computedPathsBB[robotId].first.value().getPosition(computedPathsBB[robotId].first.value().getTotalTime())).length() >
                             stp::control_constants::GO_TO_POS_ERROR_MARGIN;
            } else {
                targetBool = (targetPosition - computedPathsBB[robotId].second.value().getPosition(computedPathsBB[robotId].second.value().getTotalTime())).length() >
                             stp::control_constants::GO_TO_POS_ERROR_MARGIN;
            }
        } else return true;

        return
             //Check if the targetPosition differs by a certain margin from the end of the calculated path
             (targetBool ||

             //Check if currentPosition differs by a certain margin from the start of the calculated path
             (currentPosition - computedPaths[robotId].front()).length() > 3 * Constants::ROBOT_RADIUS() ||

             //Check if the calculated path has a collision on it
             worldObjects.getFirstCollision(world, field, computedPaths, computedVelocities, ballAvoidanceDistance, robotId, pathTimeStep, velTimeStep).has_value() ||

             //Check if the ruleSet has changed
             worldObjects.ruleset.title != ruleSet.title);
    }

    void PositionControl::trackPathBBT(int robotId, Vector2 currentPosition, Vector2 currentVelocity, rtt::BB::CommandCollision &commandCollision){
        // TODO: thoroughly test the tracking and PID values as it has not been tested yet
        if (!computedVelocities[robotId].empty()) {
            Position trackingVelocity = pathTrackingAlgorithm.trackVelocity(currentVelocity,computedVelocities[robotId],robotId,stp::PIDType::BBT);
            commandCollision.robotCommand.vel = Vector2(trackingVelocity.x,trackingVelocity.y);
            commandCollision.robotCommand.angle = trackingVelocity.rot;
        } else {
            commandCollision.robotCommand.vel = Vector2(0,0);
            commandCollision.robotCommand.angle = 0;
        }
    }

    std::pair<std::optional<BB::BBTrajectory2D>,std::optional<BB::BBTrajectory2D>>
    PositionControl::findNewPath(const rtt::world::World *world, const rtt::world::Field &field, int robotId, Vector2 &currentPosition, Vector2 &currentVelocity,
                                 std::optional<BB::CollisionData> &firstCollision, Vector2 &targetPosition, std::optional<double> ballAvoidanceDistance, double pathTimeStep, double velTimeStep) {
        // Create and score intermediate points
        std::vector<Vector2> intermediatePoints = createIntermediatePoints(field, robotId, firstCollision, targetPosition);
        // Priority queue that sorts itself from best to worst intermediate point
        std::priority_queue<std::pair<double, Vector2>, std::vector<std::pair<double, Vector2>>, std::greater<>> intermediatePointsSorted = scoreIntermediatePoints(intermediatePoints, firstCollision);
        std::pair<std::optional<BB::BBTrajectory2D>,std::optional<BB::BBTrajectory2D>> pathPair;

        BB::BBTrajectory2D pathToIntermediatePoint;
        while (!intermediatePointsSorted.empty()) {
            //TODO: Make sure that when a robot drives towards this intermediate point, it doesnt reach vel = 0.
            // So maybe instead of using the intermediatePoint as its target, use the last two points of the path towards
            // the point, calculate a drivingDirection, and extend the target beyond the point in this direction.
            pathToIntermediatePoint = BB::BBTrajectory2D(currentPosition, currentVelocity,
                                                         intermediatePointsSorted.top().second,
                                                         ai::Constants::MAX_VEL(), ai::Constants::MAX_ACC_UPPER());

            // Temporarily save current path and velocity approach
            std::vector<Vector2> previousPath = computedPaths[robotId];
            std::vector<Vector2> previousVel = computedVelocities[robotId];

            // Put the to be tested path and velocity approach in computedPaths and computedVelocities
            computedPaths[robotId] = pathToIntermediatePoint.getPathApproach(pathTimeStep);
            computedVelocities[robotId] = pathToIntermediatePoint.getVelocityApproach(velTimeStep);

            // Check path for collisions
            std::optional<BB::CollisionData> intermediatePathCollision = worldObjects.getFirstCollision(world, field, computedPaths, computedVelocities, ballAvoidanceDistance, robotId, pathTimeStep, velTimeStep);

            std::optional<BB::BBTrajectory2D> intermediateToTarget;
            // If there are no collisions, tries to calculate a path from points on the pathToIntermediatePoint to the targetPosition
            if(!intermediatePathCollision.has_value()) {
                intermediateToTarget =
                    calculatePathFromNewStart(world, field, pathToIntermediatePoint, targetPosition, ballAvoidanceDistance, robotId, pathTimeStep, velTimeStep);
            }
            if (intermediateToTarget.has_value()) {
                interface::Input::drawData(interface::Visual::PATHFINDING, intermediatePoints, Qt::green, robotId,interface::Drawing::CROSSES);
                interface::Input::drawData(interface::Visual::PATHFINDING, {firstCollision->collisionPosition},Qt::red, robotId, interface::Drawing::CROSSES);
                interface::Input::drawData(interface::Visual::PATHFINDING,pathToIntermediatePoint.getPathApproach(pathTimeStep),Qt::white, robotId,interface::Drawing::LINES_CONNECTED);
                interface::Input::drawData(interface::Visual::PATHFINDING,intermediateToTarget.value().getPathApproach(pathTimeStep),Qt::yellow, robotId, interface::Drawing::LINES_CONNECTED);

                // If a path was found return this path otherwise pop the intermediatePoint
                pathPair.first = pathToIntermediatePoint;
                pathPair.second = intermediateToTarget;
                break;
            }

            // If the path was no success put the temporarily saved path and velocity approach back in computedPaths and computedVelocities and pop the intermediate point
            computedPaths[robotId] = previousPath;
            computedVelocities[robotId] = previousVel;
            intermediatePointsSorted.pop();
        }
        return pathPair;
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
                        rtt::ai::FieldComputations::pointIsInField(field, intermediatePoint, rtt::ai::Constants::ROBOT_RADIUS())) {
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
            // The score is based on how close the point is to the collisionPoint. This scoring system was chosen such that the new path
            // deviates as little as possible from the old path as the old path was optimal in time
            intermediatePointScore = (i - firstCollision->collisionPosition).length();
            std::pair<double, Vector2> p = {intermediatePointScore, i};
            intermediatePointsSorted.push(p);
        }
        return intermediatePointsSorted;
    }

    std::optional<BB::BBTrajectory2D>
    PositionControl::calculatePathFromNewStart(const rtt::world::World *world, const rtt::world::Field &field, BB::BBTrajectory2D pathToIntermediatePoint,
                                               Vector2 &targetPosition, std::optional<double> ballAvoidanceDistance, int robotId, double pathTimeStep, double velTimeStep) {
        BB::BBTrajectory2D intermediateToTarget;
        // Increase in timeStep size to decrease calculation time
        double pathTimeStepIncrease = 2;
        pathTimeStep *= pathTimeStepIncrease;

        int numberOfTimeSteps = floor(pathToIntermediatePoint.getTotalTime() / pathTimeStep);
        for (int i = 1; i < numberOfTimeSteps; i++) {
            //TODO: Only create newStart's up to point where we dont decelerate yet
            Vector2 newStart = pathToIntermediatePoint.getPosition(i * pathTimeStep);
            Vector2 newVelocity = pathToIntermediatePoint.getVelocity(i * pathTimeStep);

            // Create intermediate path and path and velocity approach
            intermediateToTarget = BB::BBTrajectory2D(newStart, newVelocity, targetPosition, ai::Constants::MAX_VEL(), ai::Constants::MAX_ACC_UPPER());
            std::vector<Vector2> pathApproach = intermediateToTarget.getPathApproach(pathTimeStep / pathTimeStepIncrease);
            std::vector<Vector2> velApproach = intermediateToTarget.getVelocityApproach(velTimeStep);

            // Temporarily save previous path and velocity
            std::vector<Vector2> previousPath = computedPaths[robotId];
            std::vector<Vector2> previousVel = computedVelocities[robotId];

            // Erase part that comes after intermediate path
            computedPaths[robotId].erase(computedPaths[robotId].begin() + i * pathTimeStepIncrease,computedPaths[robotId].end());
            computedVelocities[robotId].erase(computedVelocities[robotId].begin() + (int)((double)i * pathTimeStep / velTimeStep) ,computedVelocities[robotId].end());

            // Insert new path and velocity
            computedPaths[robotId].insert(computedPaths[robotId].end(), pathApproach.begin(), pathApproach.end());
            computedVelocities[robotId].insert(computedVelocities[robotId].end(), velApproach.begin(), velApproach.end());

            // Check for collisions
            auto newStartCollisions = worldObjects.getFirstCollision(world, field, computedPaths, computedVelocities, ballAvoidanceDistance, robotId, pathTimeStep / pathTimeStepIncrease, velTimeStep);

            if (newStartCollisions.has_value()) {
                // If the path has a collision on it put the previous path and velocity approach back in computedPaths and computedVelocities
                computedPaths[robotId] = previousPath;
                computedVelocities[robotId] = previousVel;
                continue;
            } else {
                return intermediateToTarget;
            }
        }
        return std::nullopt;
    }

    void PositionControl::makeApproaches(int robotId, double pathTimeStep, double velTimeStep) {
            // Create a discrete approximation for the path and velocity for both BBT's
            std::vector<Vector2> secondPathPart = computedPathsBB[robotId].second.value().getPathApproach(pathTimeStep);
            std::vector<Vector2> secondVelocityPart = computedPathsBB[robotId].second.value().getVelocityApproach(velTimeStep);

            // Find at which index the second part starts for the path and the velocity
            auto it = find(computedPaths[robotId].begin(),computedPaths[robotId].end(), secondPathPart.front());
            int index = it - computedPaths[robotId].begin();
            int indexVel = (double)index * pathTimeStep / velTimeStep;

            // Erase all elements that come after the start of the second part from the first part
            computedPaths[robotId].erase(it,computedPaths[robotId].end());
            computedVelocities[robotId].erase(computedVelocities[robotId].begin() + indexVel,computedVelocities[robotId].end());

            // Insert the second part of the approximation at the end of the vectors
            computedPaths[robotId].insert(computedPaths[robotId].end(), secondPathPart.begin(), secondPathPart.end());
            computedVelocities[robotId].insert(computedVelocities[robotId].end(), secondVelocityPart.begin(), secondVelocityPart.end());
    }

}  // namespace rtt::ai::control