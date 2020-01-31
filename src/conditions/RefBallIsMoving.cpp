//
// Created by mrlukasbos on 2-5-19.
//

#include "conditions/RefBallIsMoving.h"

#include <utilities/GameStateManager.hpp>

#include "world/Ball.h"

namespace rtt::ai {

RefBallIsMoving::RefBallIsMoving(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)) {};

bt::Node::Status RefBallIsMoving::onUpdate() {
    bool ballIsLayingStill = GameStateManager::getCurrentGameState().ballPositionAtStartOfGameState.dist(ball->getPos()) < 0.05;

    if (ballIsLayingStill) {
        return Status::Failure;
    }
    return Status::Success;
}

}  // namespace rtt::ai
