//
// Created by ratoone on 18-11-19.
//

#ifndef RTT_POSITIONCONTROL_H
#define RTT_POSITIONCONTROL_H

#include "CollisionDetector.h"
#include "control/positionControl/BBTrajectories/WorldObjects.h"
#include "control/RobotCommand.h"
#include "control/positionControl/pathPlanning/NumTreesPlanning.h"
#include "control/positionControl/pathTracking/PidTracking.h"
#include "world/views/RobotView.hpp"

namespace rtt::ai::control {

/**
 * The main position control class. Use this for your robot position control
 * requirements.
 */
    class PositionControl {
    private:
        /// the distance to the target position at which the robot will stop if it
        /// detects a collision (e.g. target is inside a robot)
        static constexpr double FINAL_AVOIDANCE_DISTANCE = 4 * Constants::ROBOT_RADIUS();

        CollisionDetector collisionDetector;
        rtt::BB::WorldObjects worldObjects;
        NumTreesPlanning pathPlanningAlgorithm = NumTreesPlanning(collisionDetector);
        PidTracking pathTrackingAlgorithm;

        std::unordered_map<int, BB::BBTrajectory2D> computedPathsBB;
        std::unordered_map<int, std::vector<Vector2>> computedPaths;

    public:
        /**
         * Generates a path according to the selected planning algorithm,
         * and tracks it using the selected tracking algorithm. In the case a collision
         * is detected (using the collision detector), the path is recalculated.
         * @param field the field object, used onwards by the collision detector
         * @param robotId the ID of the robot for which the path is calculated
         * @param currentPosition the current position of the aforementioned robot
         * @param currentVelocity its velocity
         * @param targetPosition the desired position that the robot has to reach
         * @param pidType The desired PID type (intercept, regular, keeper etc.)
         * @return a RobotCommand, which can be fed directly in the output
         */
        RobotCommand computeAndTrackPath(const rtt_world::Field &field, int robotId, const Vector2 &currentPosition,
                                         const Vector2 &currentVelocity, Vector2 &targetPosition,
                                         stp::PIDType pidType);

        /**
         * Updates the robot view vector
         * @param robotPositions the position vector of the robots
         */
        void setRobotPositions(std::vector<Vector2> &robotPositions);


        /**
         * The computed path should be recalculated if: <br>
         * - it is empty (no path yet) <br>
         * - the target position changed with at least MAX_TARGET_DEVIATION <br>
         * - the robot will collide with another one by the next path point (ignored if the robot is not moving)
         * @param targetPos final target position
         * @param robotId the ID of the current robot
         * @return true if one of the above conditions are true, false otherwise
         */
        bool shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos,
                                   const Vector2 &currentVelocity, int robotId);

        /**
         * @brief Generates a path and tracks it with the old PID control (hacky). Returns also possibly
         * the location of a collision on the path if no correct path can be found
         * @param world a pointer to the current world
         * @param field the field object, used onwards by the collision detector
         * @param robotId the ID of the robot for which the path is calculated
         * @param currentPosition the current position of the aforementioned robot
         * @param currentVelocity its velocity
         * @param targetPosition the desired position that the robot has to reach
         * @param pidType The desired PID type (intercept, regular, keeper etc.)
         * @return A RobotCommand and optional with the location of the first collision on the path
         */
        rtt::BB::CommandCollision
        computeAndTrackPathBBT(const rtt::world::World *world, const rtt::world::Field &field, int robotId, Vector2 currentPosition, Vector2 currentVelocity,
                               Vector2 targetPosition, stp::PIDType pidType);

        /**
         * @brief Tries to find a new path when the current path has a collision on it. It tries this by
         * looking for paths which go to intermediate points in the area of the collision and from these
         * paths again to the target. Also draws the intermediate point and path in the interface
         * @param world the world object
         * @param field the field object, used onwards by the collision detector
         * @param robotId the ID of the robot for which the path is calculated
         * @param currentPosition the current position of the aforementioned robot
         * @param currentVelocity its velocity
         * @param firstCollision location of the first collision on the current path
         * @param targetPosition the desired position that the robot has to reach
         * @param timeStep the time between path points when approaching the path
         * @return An optional with a new path
         */
        std::optional<BB::BBTrajectory2D>
        findNewPath(const rtt::world::World *world, const rtt::world::Field &field, int robotId, Vector2 &currentPosition, Vector2 &currentVelocity,
                    std::optional<BB::CollisionData> &firstCollision, Vector2 &targetPosition,  double timeStep);

        /**
         * Creates intermediate points to make a path to. First, a pointToDrawFrom is picked by drawing a line
         * from the target position to the obstacle and extending that line further towards our currentPosition.
         * Second, make half circle of intermediatePoints pointed towards obstaclePosition, originating from pointToDrawFrom
         * @param field the field object, used onwards by the collision detector
         * @param robotId the ID of the robot for which the path is calculated
         * @param firstCollision location of the first collision on the current path
         * @param targetPosition the desired position that the robot has to reach
         * @return A vector with coordinates of the intermediate points
         */
        std::vector<Vector2>
        createIntermediatePoints(const rtt::world::Field &field, int robotId, std::optional<BB::CollisionData> &firstCollision,
                                 Vector2 &targetPosition);

        /**
         * @brief Gives each intermediate point a score for how close the point is to the collisionPosition
         * @param intermediatePoints the intermediate points for trying to find a new path
         * @param firstCollision used for scoring the points
         * @return A priority_queue to sort the points
         */
        std::priority_queue<std::pair<double, Vector2>, std::vector<std::pair<double, Vector2>>, std::greater<>>
        scoreIntermediatePoints(std::vector<Vector2> &intermediatePoints,
                               std::optional<BB::CollisionData> &firstCollision);

        //If no collision on path to intermediatePoint, create new points along this path in timeStep increments.
        //Then loop through these new points, generate paths from these to the original target and check for collisions.
        //Return the first path without collisions, or pop() this point and start checking the next one.
        /**
         * @brief Calculates a path to the targetPosition from a point on the path to an intermediate path and
         * return it if there are no collisions
         * @param world the world object
         * @param field the field object, used onwards by the collision detector
         * @param intermediatePathCollision if intermediatePathCollision has no value, return {}
         * @param pathToIntermediatePoint used for getting new start points of the BBT to the targetPosition
         * @param targetPosition the desired position that the robot has to reach
         * @param robotId the ID of the robot for which the path is calculated
         * @param timeStep time in seconds between new start points on the BBT to the intermediatePoint
         * @return optional BangBangTrajectory if a new path was found
         */
        std::optional<BB::BBTrajectory2D>
        calculatePathFromNewStart(const rtt::world::World *world, const rtt::world::Field &field,
                                  std::optional<BB::CollisionData> intermediatePathCollision, BB::BBTrajectory2D pathToIntermediatePoint,
                                  Vector2 &targetPosition, int robotId, double timeStep);
    };

}  // namespace rtt::ai::control
#endif  // RTT_POSITIONCONTROL_H
