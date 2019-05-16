//
// Created by mrlukasbos on 11-5-19.
//

#ifndef ROBOTEAM_AI_REFGAMESTATE_H
#define ROBOTEAM_AI_REFGAMESTATE_H

#include <roboteam_utils/Vector2.h>
#include "Constants.h"
#include "RuleSet.h"

namespace rtt {
namespace ai {

class RefGameState {
private:
    RefCommand commandId;
    std::string strategyName;
    std::string keeperStrategyName;
    std::string ruleSet;
    bool isfollowUpCommand;
    RefCommand followUpCommandId;
    Vector2 ballPositionAtStartOfRefGameState;

public:
    explicit RefGameState() = default;
    explicit RefGameState(RefCommand commandId, std::string strategyName, std::string keeperStrategyName, std::string ruleSet,  bool isFollowUpCommand = false, RefCommand followUpCommandId = RefCommand::UNDEFINED);

    // getters
    RefCommand getFollowUpCommandId() const;
    RefCommand getCommandId() const;
    const std::string &getStrategyName() const;
    const std::string &getKeeperStrategyName() const;
    RuleSet getRuleSet();
    bool hasFollowUpCommand() const;
    bool isFollowUpCommand() const;
    const Vector2 &getBallPositionAtStartOfRefGameState() const;
    void setBallPositionAtStartOfRefGameState(const Vector2 &ballPositionAtStartOfRefGameState);
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_REFGAMESTATE_H
