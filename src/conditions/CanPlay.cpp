
/*
 * returns SUCCESS if the ball is moving
 * USED FOR CHECKING GAME STATE CHANGES DONT USE IF YOU DONT KNOW WHEN TO
 */

#include <roboteam_ai/src/interface/api/Output.h>
#include <roboteam_ai/src/utilities/GameStateManager.hpp>
#include "../utilities/Constants.h"
#include "CanPlay.h"
#include "../world/Ball.h"

namespace rtt {
namespace ai {

CanPlay::CanPlay(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) {
};

bt::Node::Status CanPlay::onUpdate() {
    double margin = 0.05;
    if (rtt::ai::GameStateManager::getCurrentGameState().strategyName == "penalty_us_shoot_strategy") {
        margin = 0.15;
    }

    bool ballIsLayingStill = GameStateManager::getCurrentGameState().ballPositionAtStartOfGameState.dist(ball->pos) < margin;    auto refCommand = static_cast<RefCommand>(rtt::ai::GameStateManager::getRefereeData().command.command);
    if (ballIsLayingStill || (interface::Output::usesRefereeCommands() && refCommand != RefCommand::NORMAL_START)) {
        // this should keep running, because otherwise the condition would re initialize
        return Status::Failure;
    }
    return Status::Success;
}

} // ai
} // rtt
