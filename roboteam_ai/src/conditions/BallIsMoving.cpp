
/*
 * returns SUCCESS if the ball is moving
 */

#include "../utilities/Constants.h"
#include "BallIsMoving.h"

namespace rtt {
namespace ai {

BallIsMoving::BallIsMoving(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };


bt::Node::Status BallIsMoving::onUpdate() {
    Vector2 ballVel=ball->vel;

    bool ballIsLayingStill = ballVel.length() < Constants::BALL_STILL_VEL();

    if (ballIsLayingStill){
        return Status::Failure;
    }
    return Status::Success;
}

} // ai
} // rtt
