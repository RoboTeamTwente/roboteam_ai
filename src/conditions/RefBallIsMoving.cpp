//
// Created by mrlukasbos on 2-5-19.
//

#include <utilities/GameStateManager.hpp>
#include "conditions/RefBallIsMoving.h"
#include "world/Ball.h"

namespace rtt {
namespace ai {

RefBallIsMoving::RefBallIsMoving(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };


bt::Node::Status RefBallIsMoving::onUpdate() {
    bool ballIsLayingStill = GameStateManager::getCurrentGameState().ballPositionAtStartOfGameState.dist(ball->pos) < 0.05;

    if (ballIsLayingStill ){
        return Status::Failure;
    }
    return Status::Success;
}

} // ai
} // rtt
