/*
 * Returns SUCCESS if the ball is close to the goal line
 * and if it is laying still.
 * Otherwise FAILURE
 */

#include "world_old/World.h"
#include "world_old/Ball.h"
#include "conditions/BallNearOurGoalLineAndStill.h"

namespace rtt::ai {

BallNearOurGoalLineAndStill::BallNearOurGoalLineAndStill(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };


void BallNearOurGoalLineAndStill::onInitialize() {
    if (properties->hasDouble("margin")) {
        margin = properties->getDouble("margin");
    }
}

bt::Node::Status BallNearOurGoalLineAndStill::onUpdate() {
    Vector2 ballPos = world->getBall()->getPos();

    bool ballNearGoalLine = ballPos.x < (field->get_field().get(LEFT_LINE).begin.x+margin);
    bool ballIsLayingStill = Vector2(ball->getVel()).length() < Constants::BALL_STILL_VEL();

    if (ballNearGoalLine && ballIsLayingStill) {
        return Status::Success;
    } 
    return Status::Failure; 
}

} // rtt