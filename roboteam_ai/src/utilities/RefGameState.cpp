//
// Created by mrlukasbos on 11-5-19.
//

#include "RefGameState.h"

namespace rtt {
namespace ai {

RefGameState::RefGameState(
        std::string strategyName,
        std::string keeperStrategyName,
        const RuleSet &ruleSet,
        RefCommand followUpCommandId)
        :
        followUpCommandId(followUpCommandId),
        strategyName(std::move(strategyName)),
        keeperStrategyName(std::move(keeperStrategyName)),
        ruleSet(ruleSet) {}

RefCommand RefGameState::getFollowUpCommandId() const {
    return followUpCommandId;
}

const std::string &RefGameState::getStrategyName() const {
    return strategyName;
}

const std::string &RefGameState::getKeeperStrategyName() const {
    return keeperStrategyName;
}

const RuleSet &RefGameState::getRuleSet() const {
    return ruleSet;
}

} // ai
} // rtt