/*
 * returns SUCCESS if the ball is in the given defence area (standard ours)
 * AND if the ball lays still
 */
#include "conditions/BallInDefenseAreaAndStill.h"
#include "utilities/Constants.h"

namespace rtt::ai {

BallInDefenseAreaAndStill::BallInDefenseAreaAndStill(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)){};

void BallInDefenseAreaAndStill::onInitialize() {
    theirDefenceArea = properties->getBool("theirDefenceArea");
}

bt::Node::Status BallInDefenseAreaAndStill::onUpdate() {
    Vector2 ballPos = ball->get()->getPos();
    Vector2 ballVel = ball->get()->getVelocity();

    bool pointIsInDefenceArea = FieldComputations::pointIsInDefenceArea(*field, ballPos, !theirDefenceArea, 0.02, false);
    bool ballIsLayingStill = ballVel.length() < Constants::BALL_STILL_VEL();

    if (pointIsInDefenceArea && ballIsLayingStill) {
        return Status::Success;
    }
    return Status::Failure;
}

}  // namespace rtt::ai
