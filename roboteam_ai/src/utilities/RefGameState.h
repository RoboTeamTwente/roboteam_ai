//
// Created by mrlukasbos on 11-5-19.
//

#ifndef ROBOTEAM_AI_REFGAMESTATE_H
#define ROBOTEAM_AI_REFGAMESTATE_H

#include "Constants.h"
#include "RuleSet.h"

namespace rtt {
namespace ai {

class RefGameState {
private:
    std::string strategyName;
    std::string keeperStrategyName;
    RuleSet ruleSet;
    RefCommand followUpCommandId;

public:
    explicit RefGameState(std::string strategyName, std::string keeperStrategyName, const RuleSet &ruleSet,  RefCommand followUpCommandId);

    // getters
    RefCommand getFollowUpCommandId() const;
    const std::string &getStrategyName() const;
    const std::string &getKeeperStrategyName() const;
    const RuleSet &getRuleSet() const;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_REFGAMESTATE_H
