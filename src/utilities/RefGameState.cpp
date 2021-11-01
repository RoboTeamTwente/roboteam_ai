//
// Created by mrlukasbos on 11-5-19.
//

#include "utilities/RefGameState.h"

namespace rtt::ai {

RefGameState::RefGameState(RefCommand commandId, std::string strategyName, std::string ruleSetName, bool isFollowUpCommand, RefCommand followUpCommandId)
    : GameState(std::move(strategyName), std::move(ruleSetName)), commandId(commandId), isfollowUpCommand(isFollowUpCommand), followUpCommandId(followUpCommandId) {}

bool RefGameState::hasFollowUpCommand() const { return followUpCommandId != RefCommand::UNDEFINED; }

}  // namespace rtt::ai