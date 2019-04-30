
/*
 * returns SUCCESS if the ball is moving
 * USED FOR CHECKING GAME STATE CHANGES DONT USE IF YOU DONT KNOW WHEN TO
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
    auto refCommand = static_cast<RefGameState>(rtt::ai::Referee::getRefereeData().command.command);

    if (ballIsLayingStill || refCommand != RefGameState::NORMAL_START){
        return Status::Failure;
    }
    return Status::Success;
}

} // ai
} // rtt
