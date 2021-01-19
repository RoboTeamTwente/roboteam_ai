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

    struct CollisionData {
        Vector2 collisionPosition;
        Vector2 drivingDirection;
    };

    class WorldObjects {
    private:
        const rtt::ai::rtt_world::Field *field = nullptr;

        std::vector<Vector2> calculatedPaths[11] = {};
        rtt::ai::GameStateManager gameStateManager;
        rtt::ai::GameState gameState;
        rtt::ai::RuleSet ruleset;

        static world::World *world;

    public:
        WorldObjects();
        //WorldObjects(rtt::ai::GameState gameState);

        // Takes a calculated path of a robot, and checks each point along that path for (non-)stationary collisions.
        // Returns either an empty Vector or a Vector with each collision along the path.
        std::optional<CollisionData> getFirstCollision(rtt::BB::BBTrajectory2D BBTrajectory, int robotId);

        // Takes a calculated path of a robot and checks points along the path if they are outside the fieldlines if
        // the robot is not allowed to be there.
        // Adds these points and the time at which they happen to collisions and collisionTimes
        void calculateFieldCollisions(std::vector<Vector2> &collisions, std::vector<double> &collisionTimes,
                                      Vector2 &drivingDirection, const std::vector<Vector2> &pathPoints, int robotId,
                                      double timeStep);

        // Takes a calculated path of a robot and checks points along the path if they are inside the defensearea if
        // the robot is not allowed to be there.
        // Adds these points and the time at which they happen to collisions and collisionTimes
        void calculateDefenseAreaCollisions(std::vector<Vector2> &collisions, std::vector<double> &collisionTimes,
                                            Vector2 &drivingDirection, const std::vector<Vector2> &pathPoints,
                                            int robotId, double timeStep);

        // Takes a calculated path of a robot and checks points along the path if they are too close to an approximation
        // of the ball trajectory. If the play is "ball_placement_them" also checks for the path being inside the
        // balltube.
        // Adds these points and the time at which they happen to collisions and collisionTimes
        void calculateBallCollisions(std::vector<Vector2> &collisions, std::vector<double> &collisionTimes,
                                     Vector2 &drivingDirection, std::vector<Vector2> pathPoints, double timeStep);

        // Takes a calculated path of a robot and checks points along the path if they are too close to an approximation
        // of the enemy robot paths.
        // Adds these points and the time at which they happen to collisions and collisionTimes if the difference in
        // velocity between the two robots is more than 1.5 m/s and we are driving faster
        static void calculateEnemyRobotCollisions(rtt::BB::BBTrajectory2D BBTrajectory, std::vector<Vector2> &collisions,
                                           std::vector<double> &collisionTimes, Vector2 &drivingDirection,
                                           const std::vector<Vector2> &pathPoints,
                                           double timeStep);

        // Takes a path from the array of stored paths and checks points along the path if they are too close to where
        // our robots' are calculated to be at that point in time.
        // Adds these points and the time at which they happen to collisions and collisionTimes
        void calculateOurRobotCollisions(std::vector<Vector2> &collisions, std::vector<double> &collisionTimes,
                                         Vector2 &drivingDirection, const std::vector<Vector2> &pathPoints, int robotId,
                                         double timeStep);

        void storeCalculatedPath(std::vector<Vector2> points, int robotId);

        void setField(const rtt::ai::rtt_world::Field &field);

        bool canEnterDefenseArea(int robotId);

        bool canMoveOutsideField(int robotId);

        static void setWorld(world::World *world_);
    };
}

#endif //RTT_WORLDOBJECTS_H
