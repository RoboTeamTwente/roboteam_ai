
/*
 * returns SUCCESS if the ball is moving
 * USED FOR CHECKING GAME STATE CHANGES DONT USE IF YOU DONT KNOW WHEN TO
 */

#include "../utilities/Constants.h"
#include "CanPlay.h"

namespace rtt {
namespace ai {

CanPlay::CanPlay(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };


bt::Node::Status CanPlay::onUpdate() {
    Vector2 ballVel=ball->vel;

    bool ballIsLayingStill = ballVel.length() < Constants::BALL_STILL_VEL();
    auto refCommand = static_cast<RefCommand>(rtt::ai::Referee::getRefereeData().command.command);

    if (ballIsLayingStill || refCommand != RefCommand::NORMAL_START){
        return Status::Failure;
    }
    return Status::Success;
}

} // ai
} // rtt
