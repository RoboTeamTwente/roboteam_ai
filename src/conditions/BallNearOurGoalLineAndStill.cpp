/*
 * Returns SUCCESS if the ball is close to the goal line
 * and if it is laying still.
 * Otherwise FAILURE
 */

#include "conditions/BallNearOurGoalLineAndStill.h"
#include <world/Ball.h>
#include <world/World.h>

namespace rtt::ai {

BallNearOurGoalLineAndStill::BallNearOurGoalLineAndStill(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)){};

void BallNearOurGoalLineAndStill::onInitialize() {
    if (properties->hasDouble("margin")) {
        margin = properties->getDouble("margin");
    }
}

bt::Node::Status BallNearOurGoalLineAndStill::onUpdate() {
    Vector2 ballPos = world->getBall()->getPos();

    bool ballNearGoalLine = ballPos.x < ((*field).getLeftLine().begin.x + margin);
    bool ballIsLayingStill = Vector2(ball->getVel()).length() < Constants::BALL_STILL_VEL();

    if (ballNearGoalLine && ballIsLayingStill) {
        return Status::Success;
    }
    return Status::Failure;
}

}  // namespace rtt::ai