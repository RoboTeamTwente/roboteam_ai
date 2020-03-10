/*
 * returns SUCCESS if the ref state is NormalPlay. Otherwise FAILURE.
 */

#include "conditions/RefStateIsNormalPlay.h"

namespace rtt::ai {

RefStateIsNormalPlay::RefStateIsNormalPlay(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)){};

bt::Node::Status RefStateIsNormalPlay::onUpdate() {
    auto refCommand = static_cast<RefCommand>(rtt::ai::GameStateManager::getRefereeData().command());
    if (interface::Output::usesRefereeCommands() && refCommand != RefCommand::NORMAL_START) {
        return Status::Failure;
    }
    return Status::Success;
}

}  // namespace rtt::ai
