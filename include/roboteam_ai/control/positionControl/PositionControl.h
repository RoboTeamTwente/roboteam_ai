//
// Created by ratoone on 18-11-19.
//

#ifndef RTT_POSITIONCONTROL_H
#define RTT_POSITIONCONTROL_H

#include <roboteam_utils/RobotCommands.hpp>

#include "BBTrajectories/Trajectory2D.h"
#include "CollisionDetector.h"
#include "control/positionControl/BBTrajectories/WorldObjects.h"
#include "control/positionControl/pathPlanning/NumTreesPlanning.h"
#include "control/positionControl/pathTracking/BBTPathTracking.h"
#include "control/positionControl/pathTracking/DensePathTracking.h"
#include "control/positionControl/pathTracking/PidTracking.h"
#include "utilities/StpInfoEnums.h"

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
    DensePathTracking pathTrackingAlgorithm;
    BBTPathTracking pathTrackingAlgorithmBBT;

    std::unordered_map<int, Trajectory2D> computedTrajectories;
    std::unordered_map<int, std::vector<Vector2>> computedPaths;
    std::unordered_map<int, std::vector<Vector2>> computedPathsVel;
    std::unordered_map<int, std::vector<std::pair<Vector2, Vector2>>> computedPathsPosVel;

    /**
     * @brief Checks if the current path should be recalculated:
     * @param world a pointer to the current world
     * @param field the field object, used onwards by the collision detector
     * @param robotId the ID of the robot for which the path is calculated
     * @param currentPosition the current position of the aforementioned robot
     * @param currentVelocity its velocity
     * @param targetPosition the desired position that the robot has to reachsho
     * @return Boolean that is 1 if the path needs to be recalculated
     */
    bool shouldRecalculateTrajectory(const rtt::world::World *world, const rtt::world::Field &field, int robotId, Vector2 targetPosition, const Vector2 &currentPosition,
                                     ai::stp::AvoidObjects);

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
    RobotCommand computeAndTrackPath(const rtt_world::Field &field, int robotId, const Vector2 &currentPosition, const Vector2 &currentVelocity, Vector2 &targetPosition,
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
    bool shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos, const Vector2 &currentVelocity, int robotId);

    /**
     * @brief Generates a collision-free trajectory and tracks it. Returns also possibly
     * the location of a collision on the path if no correct path can be found
     * @param world a pointer to the current world
     * @param field the field object, used onwards by the collision detector
     * @param robotId the ID of the robot for which the path is calculated
     * @param currentPosition the current position of the aforementioned robot
     * @param currentVelocity its velocity
     * @param targetPosition the desired position that the robot has to reach
     * @param maxRobotVelocity the maximum velocity that the robot is allowed to have
     * @param pidType The desired PID type (intercept, regular, keeper etc.)
     * @return A RobotCommand and optional with the location of the first collision on the path
     */
    rtt::BB::CommandCollision computeAndTrackTrajectory(const rtt::world::World *world, const rtt::world::Field &field, int robotId, Vector2 currentPosition,
                                                        Vector2 currentVelocity, Vector2 targetPosition, double maxRobotVelocity, stp::PIDType pidType,
                                                        stp::AvoidObjects avoidObjects);

    /**
     * @brief Tries to find a new trajectory when the current path has a collision on it. It tries this by
     * looking for trajectories which go to intermediate points in the area of the collision and from these
     * paths again to the target. Also draws the intermediate point and path in the interface
     * @param world the world object
     * @param field the field object, used onwards by the collision detector
     * @param robotId the ID of the robot for which the path is calculated
     * @param currentPosition the current position of the aforementioned robot
     * @param currentVelocity its velocity
     * @param firstCollision location of the first collision on the current path
     * @param targetPosition the desired position that the robot has to reach
     * @param maxRobotVelocity the maximum velocity the robot is allowed to have
     * @param timeStep the time between path points when approaching the path
     * @return An optional with a new path
     */
    std::optional<Trajectory2D> findNewTrajectory(const rtt::world::World *world, const rtt::world::Field &field, int robotId, Vector2 &currentPosition, Vector2 &currentVelocity,
                                                  std::optional<BB::CollisionData> &firstCollision, Vector2 &targetPosition, double maxRobotVelocity, double timeStep,
                                                  stp::AvoidObjects AvoidObjects);

    // If no collision on trajectory to intermediatePoint, create new points along this trajectory in timeStep increments.
    // Then loop through these new points, generate trajectories from these to the original target and check for collisions.
    // Return the first trajectory without collisions, or pop() this point and start checking the next one.
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
     * @return optional Trajectory if a new path was found
     */
    std::optional<Trajectory2D> calculateTrajectoryAroundCollision(const rtt::world::World *world, const rtt::world::Field &field,
                                                                   std::optional<BB::CollisionData> &intermediatePathCollision, Trajectory2D trajectoryToIntermediatePoint,
                                                                   Vector2 &targetPosition, int robotId, double maxRobotVelocity, double timeStep, stp::AvoidObjects avoidObjects);

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
    std::vector<Vector2> createIntermediatePoints(const rtt::world::Field &field, int robotId, std::optional<BB::CollisionData> &firstCollision, Vector2 &targetPosition);

    /**
     * @brief Gives each intermediate point a score for how close the point is to the collisionPosition
     * @param intermediatePoints the intermediate points for trying to find a new path
     * @param firstCollision used for scoring the points
     * @return A priority_queue to sort the points
     */
    std::priority_queue<std::pair<double, Vector2>, std::vector<std::pair<double, Vector2>>, std::greater<>> scoreIntermediatePoints(
        std::vector<Vector2> &intermediatePoints, std::optional<BB::CollisionData> &firstCollision);
};

}  // namespace rtt::ai::control
#endif  // RTT_POSITIONCONTROL_H
