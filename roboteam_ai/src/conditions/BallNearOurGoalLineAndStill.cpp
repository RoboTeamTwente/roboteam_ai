//
// Created by rolf on 20-2-19.
//

#include "BallNearOurGoalLineAndStill.h"
namespace rtt{
namespace ai{
BallNearOurGoalLineAndStill::BallNearOurGoalLineAndStill(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

void BallNearOurGoalLineAndStill::onInitialize() {
    if (properties->hasDouble("margin")) {
        margin = properties->getDouble("margin");
    }
}

bt::Node::Status BallNearOurGoalLineAndStill::onUpdate() {
    Vector2 ballPos = world::world->getBall()->pos;
    if (ballPos.x<(world::field->get_field().left_line.begin.x+margin)&&Vector2(ball->vel).length()<Constants::BALL_STILL_VEL()) {
        return Status::Success;
    } else {
        return Status::Failure;
    }
}

std::string BallNearOurGoalLineAndStill::node_name() {return "BallNearOurGoalLineAndStill";}
}
}