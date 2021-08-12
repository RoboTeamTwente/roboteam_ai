//
// Created by floris on 15-11-20.
//

#ifndef RTT_WORLDOBJECTS_H
#define RTT_WORLDOBJECTS_H

#include <control/ControlUtils.h>
#include "world/FieldComputations.h"
#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"
#include <utilities/GameStateManager.hpp>
#include "control/RobotCommand.h"


namespace rtt::BB {

    /**
     * @memberof obstaclePosition position of the obstacle
     * @memberof collisionPosition position robot shouldn't come
     * @memberof collisionTime number of seconds from now that the collision will occur
     * @memberof collisionName the name of what causes the collision
     */
    struct CollisionData {
        Vector2 obstaclePosition;
        Vector2 collisionPosition;
        double collisionTime;
        std::string collisionName;
    };

    /**
     * @brief struct used for returning a command for pathtracking and
     * information about a collision which can be used STP
     * @memberof robotCommand
     * @memberof collision
     */
    struct CommandCollision {
        RobotCommand robotCommand;
        std::optional<CollisionData> collisionData;
    };

    /**
     * @author Jaro Kuiken & Floris Hoek
     * @brief class that checks if a bang bang trajectory has collisions on it by taking into account:
     * current ruleset, predicted ball path, predicted enemy robot path and already calculated robot paths
     */
    class WorldObjects {
    private:
        rtt::ai::GameStateManager gameStateManager;
        rtt::ai::GameState gameState;

    public:
        rtt::ai::RuleSet ruleset;

        WorldObjects();

        /**
         * Takes a BangBangTrajectory of a robot and checks the path in certain intervals for collisions
         * @brief Calculates the position of the first collision and the obstacle position on a BangBangTrajectory
         * @param world the world object used for information about the robots
         * @param field used for checking collisions with the field
         * @param BBTrajectory the trajectory to check for collisions
         * @param computedPaths the paths of our robots
         * @param ballAvoidanceDistance the distance the robot should keep from the ball
         * @param robotId
         * @return optional with rtt::BB::CollisionData
         */
        std::optional<CollisionData> getFirstCollision(const rtt::world::World *world, const rtt::world::Field &field,
                                                       const std::unordered_map<int, std::vector<Vector2>> &computedPaths,
                                                       const std::unordered_map<int, std::vector<Vector2>> &computedVelocities,
                                                       std::optional<double> ballAvoidanceDistance,
                                                       int robotId, const double pathTimeStep, const double velTimeStep);

        /**
         * @brief Takes a calculated path of a robot and checks points along the path if they are outside the
         * fieldlines if not allowed there. Adds these points and the time to collisionDatas and collisionTimes
         * @param field Used for information about the field
         * @param collisionDatas, std::vector which rtt::BB::CollisionData can be added to
         * @param pathPoints, std::vector with path points
         * @param robotId
         * @param timeStep in seconds
         */
        void calculateFieldCollisions(const rtt::world::Field &field, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints,
                                      int robotId, double timeStep);

        /**
         * @brief Takes a calculated path and checks points along the path if they are inside the defensearea if
         * the robot is not allowed to be there. Adds these points and the time to collisionDatas and collisionTimes
         * @param field Used for information about the field
         * @param collisionDatas, std::vector which rtt::BB::CollisionData can be added to
         * @param pathPoints, std::vector with path points
         * @param robotId
         * @param timeStep in seconds
         */
        void calculateDefenseAreaCollisions(const rtt::world::Field &field, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints,
                                            int robotId, double timeStep);

        /**
         * @brief Takes a calculated path of a robot and checks points along the path if they are too close to an
         * approximation of the ball trajactory. If the play is "ball_placement_them" also checks for the path
         * being inside the balltube. If a ballAvoidanceDistance is passed it will overwrite the distance from the ruleset.
         * Adds these points and the time to collisionDatas and collisionTimes
         * @param world Used for information about the ball
         * @param collisionDatas, std::vector which rtt::BB::CollisionData can be added to
         * @param pathPoints, std::vector with path points
         * @param ballAvoidanceDistance the distance the robot should keep from the ball
         * @param timeStep in seconds
         */
        void calculateBallCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas,std::vector<Vector2> pathPoints,
                                     std::optional<double> ballAvoidanceDistance, double timeStep);

        /**
         * @brief Takes a calculated path of a robot and checks points along the path if they are too close to an
         * approximation of the enemy robot paths. Adds these points and the time to collisionDatas and collisionTimes if
         * the difference in velocity between the two robots is more than 1.5 ms/s and we are driving faster
         * @param world Used for information about the enemy robots
         * @param BBTrajectory BBTrajectory which is used for getting the velocity of the robot
         * @param collisionDatas, std::vector which rtt::BB::CollisionData can be added to
         * @param pathPoints, std::vector with path points
         * @param timeStep in seconds
         */
        void calculateEnemyRobotCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas, const std::vector<Vector2> &pathPoints,
                                           const std::vector<Vector2> &velocityPoints, double pathTimeStep, double velTimeStep);

        /**
         * @brief Takes a path from the array of stored paths and checks points along the path if they are too close to
         * where our robots are calculated to be at that point in time. Adds these points and the time to collisionDatas
         * and collisionTimes
         * @param world Used for information about our robots
         * @param collisionDatas, std::vector which rtt::BB::CollisionData can be added to
         * @param pathPoints, std::vector with path points
         * @param computedPaths The paths of our own robots
         * @param robotId
         * @param timeStep in seconds
         */
        void calculateOurRobotCollisions(const rtt::world::World *world, std::vector<CollisionData> &collisionDatas,const std::vector<Vector2> &pathPoints,
                                         const std::unordered_map<int, std::vector<Vector2>> &computedPaths, int robotId, double timeStep);

        /**
         * @brief returns true if robotId is equal to the keeperId and otherwise checks the ruleSet if the robot is allowed to enter the defense area
         * @param robotId
         * @return
         */
        bool canEnterDefenseArea(int robotId);

        /**
         * @brief returns true if robotId is equal to the keeperId and otherwise checks the ruleSet if the robot is allowed to go out of the field
         * @param robotId
         * @return
         */
        bool canMoveOutsideField(int robotId);

        /**
         * @brief inserts the data in collisionDatas such that it is ordered from lowest to highest collionTime
         * @param collisionDatas the vector in which collisionData needs to be inserted
         * @param collisionData the to be inserted data
         */
        void insertCollisionData(std::vector<CollisionData> &collisionDatas, const CollisionData &collisionData);
    };
}

#endif //RTT_WORLDOBJECTS_H
