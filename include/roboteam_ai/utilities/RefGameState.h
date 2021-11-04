//
// Created by mrlukasbos on 11-5-19.
//

#ifndef ROBOTEAM_AI_REFGAMESTATE_H
#define ROBOTEAM_AI_REFGAMESTATE_H

#include <roboteam_utils/Vector2.h>

#include "Constants.h"
#include "GameState.h"
#include "RuleSet.h"

namespace rtt::ai {

struct RefGameState : public GameState {
    RefCommand commandId;
    bool isfollowUpCommand;
    RefCommand followUpCommandId;
    RefGameState() = default;
    RefGameState(RefCommand commandId, std::string strategyName, std::string ruleSet, bool isFollowUpCommand = false, RefCommand followUpCommandId = RefCommand::UNDEFINED);
    bool hasFollowUpCommand() const;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_REFGAMESTATE_H
