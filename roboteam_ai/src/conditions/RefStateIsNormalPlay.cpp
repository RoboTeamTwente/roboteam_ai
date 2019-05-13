//
// Created by mrlukasbos on 2-5-19.
//

#include <roboteam_ai/src/interface/api/Input.h>
#include <roboteam_ai/src/interface/api/Output.h>
#include "RefStateIsNormalPlay.h"
#include "../utilities/Constants.h"

namespace rtt {
namespace ai {

RefStateIsNormalPlay::RefStateIsNormalPlay(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

bt::Node::Status RefStateIsNormalPlay::onUpdate() {
    auto refCommand = static_cast<RefGameState>(rtt::ai::Referee::getRefereeData().command.command);

    if (interface::Output::usesRefereeCommands() && refCommand != RefGameState::NORMAL_START){
        return Status::Failure;
    }
    return Status::Success;
}

} // ai
} // rtt
