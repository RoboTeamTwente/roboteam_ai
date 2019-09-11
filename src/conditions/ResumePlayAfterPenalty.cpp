//
// Created by rolf on 15-6-19.
//
#include <utilities/GameStateManager.hpp>
#include <world/Field.h>
#include "conditions/ResumePlayAfterPenalty.h"
#include "world/Ball.h"
#include "control/ControlUtils.h"

namespace rtt{
namespace ai{
ResumePlayAfterPenalty::ResumePlayAfterPenalty(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) {
}

bt::Node::Status ResumePlayAfterPenalty::onUpdate() {
    bool ballHasMoved= GameStateManager::getCurrentGameState().ballPositionAtStartOfGameState.dist(ball->pos) > 0.05;
    bool ballIsLayingStill = (Vector2(ball->vel)).length() < Constants::BALL_STILL_VEL();


    if (!ballShot&&!ballIsLayingStill&&ballHasMoved) {
        ballShot=true;
    }
    if (ballShot){
        ticks++;
    }
    std::cout<<ticks<<std::endl;
    if (ticks>0.7*Constants::TICK_RATE()){
        ticks=0;
        ballShot=false;
        return Status::Success;
    }
    return Status::Failure;
}
}
}