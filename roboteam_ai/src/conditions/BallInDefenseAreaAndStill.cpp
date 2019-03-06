//
// Created by rolf on 14-1-19.
//

#include "BallInDefenseAreaAndStill.h"
#include "../utilities/World.h"
#include "../utilities/Field.h"
#include "../utilities/Constants.h"

namespace rtt {
namespace ai {

BallInDefenseAreaAndStill::BallInDefenseAreaAndStill(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

void BallInDefenseAreaAndStill::onInitialize() {
    if (properties->hasBool("theirDefenceArea")) {
        theirDefenceArea = properties->getBool("theirDefenceArea");
    }
    else {theirDefenceArea=false;}
}

bt::Node::Status BallInDefenseAreaAndStill::onUpdate() {
    Vector2 ballPos;
    auto ball=World::getBall();
    if (ball){
        ballPos=ball->pos;
    }
    else return Status::Failure;
    Vector2 ballVel=ball->vel;
    if (Field::pointIsInDefenceArea(ballPos,!theirDefenceArea)&&(ballVel.length()<Constants::BALL_STILL_VEL())){
        return Status::Success;
    }
    else{
        return Status::Failure;
    }
}

std::string BallInDefenseAreaAndStill::node_name() {return "BallInDefenseAreaAndStill";}

}
}
