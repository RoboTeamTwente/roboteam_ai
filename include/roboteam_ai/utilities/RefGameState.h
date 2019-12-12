//
// Created by mrlukasbos on 11-5-19.
//

#ifndef ROBOTEAM_AI_REFGAMESTATE_H
#define ROBOTEAM_AI_REFGAMESTATE_H

#include <roboteam_utils/Vector2.h>
#include "Constants.h"
#include "RuleSet.h"
#include "GameState.h"

namespace rtt {
namespace ai {

struct RefGameState : public GameState {
    RefCommand commandId;
    bool isfollowUpCommand;
    RefCommand followUpCommandId;
    RefGameState() = default;
    RefGameState(RefCommand commandId, std::string strategyName, std::string keeperStrategyName, std::string ruleSet,  bool isFollowUpCommand = false, RefCommand followUpCommandId = RefCommand::UNDEFINED);
    bool hasFollowUpCommand() const;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_REFGAMESTATE_H
