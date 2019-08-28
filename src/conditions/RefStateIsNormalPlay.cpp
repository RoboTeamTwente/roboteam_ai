//
// Created by mrlukasbos on 2-5-19.
//

#include <include/roboteam_ai/interface/api/Input.h>
#include <include/roboteam_ai/interface/api/Output.h>
#include <include/roboteam_ai/utilities/GameStateManager.hpp>
#include "include/roboteam_ai/conditions/RefStateIsNormalPlay.h"
#include "include/roboteam_ai/utilities/Constants.h"

namespace rtt {
namespace ai {

RefStateIsNormalPlay::RefStateIsNormalPlay(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

bt::Node::Status RefStateIsNormalPlay::onUpdate() {
    auto refCommand = static_cast<RefCommand>(rtt::ai::GameStateManager::getRefereeData().command());
    if (interface::Output::usesRefereeCommands() && refCommand != RefCommand::NORMAL_START){
        return Status::Failure;
    }
    return Status::Success;
}

} // ai
} // rtt
