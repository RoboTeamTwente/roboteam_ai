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
    RefCommand commandId;
    std::string strategyName;
    std::string keeperStrategyName;
    RuleSet ruleSet;
    bool isfollowUpCommand;
    RefCommand followUpCommandId;

public:
    explicit RefGameState() = default;
    explicit RefGameState(RefCommand commandId, std::string strategyName, std::string keeperStrategyName, const RuleSet &ruleSet,  bool isFollowUpCommand = false, RefCommand followUpCommandId = RefCommand::UNDEFINED);

    // getters
    RefCommand getFollowUpCommandId() const;
    RefCommand getCommandId() const;
    const std::string &getStrategyName() const;
    const std::string &getKeeperStrategyName() const;
    const RuleSet &getRuleSet() const;
    bool hasFollowUpCommand() const;
    bool isFollowUpCommand() const;

};

} // ai
} // rtt

#endif //ROBOTEAM_AI_REFGAMESTATE_H
