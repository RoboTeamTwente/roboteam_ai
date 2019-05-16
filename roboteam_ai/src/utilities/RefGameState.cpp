//
// Created by mrlukasbos on 11-5-19.
//

#include "RefGameState.h"
#include "Referee.hpp"

namespace rtt {
namespace ai {

RefGameState::RefGameState(RefCommand commandId, std::string strategyName, std::string keeperStrategyName, std::string ruleSet, bool isFollowUpCommand, RefCommand followUpCommandId)
: commandId(commandId), strategyName(std::move(strategyName)), keeperStrategyName(std::move(keeperStrategyName)), ruleSet(ruleSet), isfollowUpCommand(isFollowUpCommand), followUpCommandId(followUpCommandId)
    {}

RefCommand RefGameState::getCommandId() const {
    return commandId;
}

RefCommand RefGameState::getFollowUpCommandId() const {
    return followUpCommandId;
}

const std::string &RefGameState::getStrategyName() const {
    return strategyName;
}

const std::string &RefGameState::getKeeperStrategyName() const {
    return keeperStrategyName;
}

RuleSet RefGameState::getRuleSet() {
    auto set = Referee::getRuleSetByName(ruleSet);
    return set;
}

bool RefGameState::hasFollowUpCommand() const {
    return getFollowUpCommandId() != RefCommand::UNDEFINED;
}

bool RefGameState::isFollowUpCommand() const {
    return isfollowUpCommand;
}

const Vector2 &RefGameState::getBallPositionAtStartOfRefGameState() const {
    return ballPositionAtStartOfRefGameState;
}

void RefGameState::setBallPositionAtStartOfRefGameState(const Vector2 &ballPositionAtStartOfRefGameState) {
    RefGameState::ballPositionAtStartOfRefGameState = ballPositionAtStartOfRefGameState;
}


} // ai
} // rtt