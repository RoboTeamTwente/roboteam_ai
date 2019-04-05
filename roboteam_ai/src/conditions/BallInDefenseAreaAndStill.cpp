//
// Created by rolf on 14-1-19.
//

#include "BallInDefenseAreaAndStill.h"
#include "../utilities/Constants.h"

namespace rtt {
namespace ai {

BallInDefenseAreaAndStill::BallInDefenseAreaAndStill(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

void BallInDefenseAreaAndStill::onInitialize() {
    theirDefenceArea = properties->getBool("theirDefenceArea");
    outsideField = properties->getBool("outsideField");
}

bt::Node::Status BallInDefenseAreaAndStill::onUpdate() {
    Vector2 ballPos = ball->pos;
    Vector2 ballVel=ball->vel;

    if (world::field->pointIsInDefenceArea(ballPos,!theirDefenceArea, outsideField)&&(ballVel.length()<Constants::BALL_STILL_VEL())){
        return Status::Success;
    }
    else{
        return Status::Failure;
    }
}

std::string BallInDefenseAreaAndStill::node_name() {return "BallInDefenseAreaAndStill";}

} // ai
} // rtt
