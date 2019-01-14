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

void BallInDefenseAreaAndStill::initialize() {
    currentTick=0;
    maxTick= static_cast<int>(floor(constants::BALL_STILL_TIME*constants::tickRate));
    theirDefenceArea=properties->getBool("theirDefenceArea");
}
bt::Node::Status BallInDefenseAreaAndStill::update() {
    Vector2 ballPos;
    auto ball=World::getBall();
    if (ball){
        ballPos=ball->pos;
    }
    else return Status::Failure;
    Vector2 ballVel=ball->vel;
    if (Field::pointIsInDefenceArea(ballPos,!theirDefenceArea)&&(ballVel.length()<constants::BALL_STILL_VEL)){
        currentTick++;
    }
    else{
        return Status::Failure;
    }
    if (currentTick>=maxTick){
        return Status::Success;
    }
    return Status::Running;
}

std::string BallInDefenseAreaAndStill::node_name() {return "BallInDefenseAreaAndStill";}

}
}
