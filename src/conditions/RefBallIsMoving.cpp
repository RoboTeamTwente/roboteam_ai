/*
 * returns SUCCESS if the ball is moving. Otherwise FAILURE.
 */

#include "conditions/RefBallIsMoving.h"

namespace rtt::ai {

RefBallIsMoving::RefBallIsMoving(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)){};

bt::Node::Status RefBallIsMoving::onUpdate() {
    bool ballIsLayingStill = GameStateManager::getCurrentGameState().ballPositionAtStartOfGameState.dist(ball->get()->getPos()) < 0.05;

    if (ballIsLayingStill) {
        return Status::Failure;
    }
    return Status::Success;
}

}  // namespace rtt::ai
