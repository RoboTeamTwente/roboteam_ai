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
        const rtt::ai::rtt_world::Field* field = nullptr;
        rtt::ai::GameStateManager gameStateManager;

        rtt::ai::GameState gameState = rtt::ai::GameStateManager::getCurrentGameState();
        rtt::ai::RuleSet ruleset = gameState.getRuleSet();
        static rtt::world::ball::Ball ball_;
        static std::vector<rtt::world::view::RobotView> robots;

    public:
        WorldObjects();
        //WorldObjects(rtt::ai::GameState gameState);

        // Returns true if there is a collision
        std::vector<Vector2> collisionChecker(rtt::BB::BBTrajectory2D BBTrajectory,int robotId);

        void setField(const rtt::ai::rtt_world::Field& field);
        static void setBall(rtt::world::ball::Ball ball);

        bool canEnterDefenseArea(int robotId);
        bool canMoveOutsideField(int robotId);

        static void setRobotPositions(std::vector<rtt::world::view::RobotView> robots_);
    };
}

#endif //RTT_WORLDOBJECTS_H
