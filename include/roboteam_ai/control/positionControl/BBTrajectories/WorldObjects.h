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


    class WorldObjects {
    private:
        const rtt::ai::rtt_world::Field *field = nullptr;

        rtt::ai::GameStateManager gameStateManager;
        rtt::ai::GameState gameState;
        rtt::ai::RuleSet ruleset;

        static world::World *world;

    public:
        WorldObjects();
        //WorldObjects(rtt::ai::GameState gameState);

        // Takes a calculated path of a robot, and checks each point along that path for (non-)stationary collisions.
        // Returns either an empty Vector or a Vector with each collision along the path.
        std::vector<Vector2> collisionChecker(rtt::BB::BBTrajectory2D BBTrajectory, int robotId);

        void calculateFieldCollisions(std::vector<Vector2> collisions, std::vector<double> collisionTimes,
                                      const std::vector<Vector2> &pathPoints, int robotId, double timeStep);

        void calculateDefenseAreaCollisions(std::vector<Vector2> collisions, std::vector<double> collisionTimes,
                                            const std::vector<Vector2> &pathPoints, int robotId, double timeStep);

        void calculateBallCollisions(std::vector<Vector2> collisions, std::vector<double> collisionTimes,
                                     std::vector<Vector2> pathPoints, double timeStep);

        void
        calculateEnemyRobotCollisions(rtt::BB::BBTrajectory2D BBTrajectory, std::vector<Vector2> collisions, std::vector<double> collisionTimes,
                                      std::vector<Vector2> pathPoints, double timeStep);

        void setField(const rtt::ai::rtt_world::Field &field);

        bool canEnterDefenseArea(int robotId);

        bool canMoveOutsideField(int robotId);

        static void setWorld(world::World *world_);

    };
}

#endif //RTT_WORLDOBJECTS_H
