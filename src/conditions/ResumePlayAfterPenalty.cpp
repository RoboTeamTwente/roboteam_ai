/*
 * returns SUCCESS if ball is shot and wait is done. Otherwise FAILURE.
 */

#include "conditions/ResumePlayAfterPenalty.h"

namespace rtt::ai {
ResumePlayAfterPenalty::ResumePlayAfterPenalty(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)) {}

bt::Node::Status ResumePlayAfterPenalty::onUpdate() {
    bool ballHasMoved = GameStateManager::getCurrentGameState().ballPositionAtStartOfGameState.dist(ball->get()->getPos()) > 0.05;
    bool ballIsLayingStill = (Vector2(ball->get()->getVelocity())).length() < Constants::BALL_STILL_VEL();

    if (!ballShot && !ballIsLayingStill && ballHasMoved) {
        ballShot = true;
    }
    if (ballShot) {
        ticks++;
    }
    if (ticks > 0.7 * Constants::TICK_RATE()) {
        ticks = 0;
        ballShot = false;
        return Status::Success;
    }
    return Status::Failure;
}
}  // namespace rtt::ai