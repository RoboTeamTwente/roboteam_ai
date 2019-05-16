//
// Created by mrlukasbos on 2-5-19.
//

#include "ballIsMoving.h"


namespace rtt {
namespace ai {

ballIsMoving::ballIsMoving(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };


bt::Node::Status ballIsMoving::onUpdate() {
    bool ballIsLayingStill = GameStateManager::getCurrentGameState().ballPositionAtStartOfGameState.dist(ball->pos) < 0.05;

    if (ballIsLayingStill ){
        return Status::Failure;
    }
    return Status::Success;
}

} // ai
} // rtt
