/*
 * returns SUCCESS if the ball is in the given defence area (standard ours) 
 * AND if the ball lays still
 */

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

    bool pointIsInDefenceArea = world::field->pointIsInDefenceArea(ballPos, !theirDefenceArea, outsideField);
    bool ballIsLayingStill = ballVel.length() < Constants::BALL_STILL_VEL();

    if (pointIsInDefenceArea && ballIsLayingStill){
        return Status::Success;
    }
    return Status::Failure;
}

} // ai
} // rtt
