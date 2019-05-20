//
// Created by mrlukasbos on 11-5-19.
//

#include "RefGameState.h"
#include "GameStateManager.hpp"

namespace rtt {
namespace ai {

RefGameState::RefGameState(RefCommand commandId, std::string strategyName, std::string keeperStrategyName, std::string ruleSetName, bool isFollowUpCommand, RefCommand followUpCommandId)
: GameState(std::move(strategyName), std::move(keeperStrategyName), std::move(ruleSetName)),
    commandId(commandId),
    isfollowUpCommand(isFollowUpCommand),
    followUpCommandId(followUpCommandId)
    {}

bool RefGameState::hasFollowUpCommand() const {
    return followUpCommandId != RefCommand::UNDEFINED;
}


} // ai
} // rtt