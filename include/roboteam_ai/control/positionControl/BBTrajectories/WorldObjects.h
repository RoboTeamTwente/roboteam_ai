//
// Created by floris on 15-11-20.
//

#ifndef RTT_WORLDOBJECTS_H
#define RTT_WORLDOBJECTS_H

#include <include/roboteam_ai/control/ControlUtils.h>
#include "world/FieldComputations.h"
#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"
#include <include/roboteam_ai/utilities/GameStateManager.hpp>


namespace rtt::BB {

    /**
     * @memberof obstaclePosition, position of the obstacle
     * @memberof collisionPosition, position robot shouldn't come
     */
    struct CollisionData {
        Vector2 obstaclePosition;
        Vector2 collisionPosition;
    };

    class WorldObjects {
    private:
        const rtt::ai::rtt_world::Field *field = nullptr;

        std::vector<Vector2> calculatedPaths[11] = {}; // Holds the paths of the different robots
        rtt::ai::GameStateManager gameStateManager;
        rtt::ai::GameState gameState;
        rtt::ai::RuleSet ruleset;

        static world::World *world;

    public:
        WorldObjects();

        /**
         * Takes a BangBangTrajectory of a robot and checks the path in certain intervals for collisions
         * @brief Calculates the position of the first collision and the obstacle position on a BangBangTrajectory
         * @param BBTrajectory
         * @param robotId
         * @return optional with rtt::BB::CollisionData
         */
        std::optional<CollisionData> getFirstCollision(rtt::BB::BBTrajectory2D BBTrajectory, int robotId);

        /**
         * @brief Takes a calculated path of a robot and checks points along the path if they are outside the
         * fieldlines if not allowed there. Adds these points and the time to collisionDatas and collisionTimes
         * @param collisionDatas, std::vector which rtt::BB::CollisionData can be added to
         * @param collisionTimes, std::vector which times can be added to
         * @param pathPoints, std::vector with path points
         * @param robotId
         * @param timeStep in seconds
         */
        void calculateFieldCollisions(std::vector<CollisionData> &collisionDatas, std::vector<double> &collisionTimes,
                                      const std::vector<Vector2> &pathPoints, int robotId, double timeStep);

        /**
         * @brief Takes a calculated path and checks points along the path if they are inside the defensearea if
         * the robot is not allowed to be there. Adds these points and the time to collisionDatas and collisionTimes
         * @param collisionDatas, std::vector which rtt::BB::CollisionData can be added to
         * @param collisionTimes, std::vector which times can be added to
         * @param pathPoints, std::vector with path points
         * @param robotId
         * @param timeStep in seconds
         */
        void calculateDefenseAreaCollisions(std::vector<CollisionData> &collisionDatas, std::vector<double> &collisionTimes,
                                            const std::vector<Vector2> &pathPoints, int robotId, double timeStep);

        /**
         * @brief Takes a calculated path of a robot and checks points along the path if they are too close to an
         * approximation of the ball trajactory. If the play is "ball_placement_them" also checks for the path
         * being inside the balltube. Adds these points and the time to collisionDatas and collisionTimes
         * @param collisionDatas, std::vector which rtt::BB::CollisionData can be added to
         * @param collisionTimes, std::vector which times can be added to
         * @param pathPoints, std::vector with path points
         * @param timeStep in seconds
         */
        void calculateBallCollisions(std::vector<CollisionData> &collisionDatas, std::vector<double> &collisionTimes,
                                     std::vector<Vector2> pathPoints, double timeStep);

        /**
         * @brief Takes a calculated path of a robot and checks points along the path if they are too close to an
         * approximation of the enemy robot paths. Adds these points and the time to collisionDatas and collisionTimes if
         * the difference in velocity between the two robots is more than 1.5 ms/s and we are driving faster
         * @param BBTrajectory BBTrajectory which is used for getting the velocity of the robot
         * @param collisionDatas, std::vector which rtt::BB::CollisionData can be added to
         * @param collisionTimes, std::vector which times can be added to
         * @param pathPoints, std::vector with path points
         * @param timeStep in seconds
         */
        void calculateEnemyRobotCollisions(rtt::BB::BBTrajectory2D BBTrajectory, std::vector<CollisionData> &collisionDatas,
                                           std::vector<double> &collisionTimes, const std::vector<Vector2> &pathPoints,
                                           double timeStep);

        /**
         * @brief Takes a path from the array of stored paths and checks points along the path if they are too close to
         * where our robots are calculated to be at that point in time. Adds these points and the time to collisionDatas
         * and collisionTimes
         * @param collisionDatas, std::vector which rtt::BB::CollisionData can be added to
         * @param collisionTimes, std::vector which times can be added to
         * @param pathPoints, std::vector with path points
         * @param robotId
         * @param timeStep in seconds
         */
        void calculateOurRobotCollisions(std::vector<CollisionData> &collisionDatas, std::vector<double> &collisionTimes,
                                         const std::vector<Vector2> &pathPoints, int robotId, double timeStep);

        void setField(const rtt::ai::rtt_world::Field &field);

        // Checks if a specified robot can enter the defense area
        bool canEnterDefenseArea(int robotId);

        // Checks if a specified robot can move out of the field
        bool canMoveOutsideField(int robotId);

        static void setWorld(world::World *world_);
    };
}

#endif //RTT_WORLDOBJECTS_H
