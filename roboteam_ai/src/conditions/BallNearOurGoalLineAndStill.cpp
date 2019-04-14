/*
 * Returns SUCCESS if the ball is close to the goal line
 * and if it is laying still.
 * Otherwise FAILURE
 */

#include "BallNearOurGoalLineAndStill.h"

namespace rtt {
namespace ai {

BallNearOurGoalLineAndStill::BallNearOurGoalLineAndStill(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };


void BallNearOurGoalLineAndStill::onInitialize() {
    if (properties->hasDouble("margin")) {
        margin = properties->getDouble("margin");
    }
}

bt::Node::Status BallNearOurGoalLineAndStill::onUpdate() {
    Vector2 ballPos = world::world->getBall()->pos;

    bool ballNearGoalLine = ballPos.x < (world::field->get_field().left_line.begin.x+margin);
    bool ballIsLayingStill = Vector2(ball->vel).length()<Constants::BALL_STILL_VEL(); 

    if (ballNearGoalLine && ballIsLayingStill) {
        return Status::Success;
    } 
    return Status::Failure; 
}