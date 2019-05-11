/*
 * Created by mrlukasbos on 9-11-18.
 *
 * Set the refgame state according to referee commands.
 */

#ifndef ROBOTEAM_AI_STRATEGYMANAGER_H
#define ROBOTEAM_AI_STRATEGYMANAGER_H

#include <iostream>
#include <map>
#include "Referee.hpp"
#include "Constants.h"
#include "RefGameState.h"

namespace rtt {
namespace ai {

class StrategyManager {
public:
    explicit StrategyManager();
    const RefGameState &getCurrentRefGameState() const;
    void setCurrentRefGameState(RefCommand command);

private:
    RefGameState currentRefGameState;
    std::map<RefCommand, RefGameState> gameStates;

    // Basic rulesets for rule compliance
    const RuleSet DEFAULT_RULESET           = {8.0, 1.5, 6.5, false, true, true};
    const RuleSet HALT_RULESET              = {0.0, 0.0, 0.0, true, true, true};
    const RuleSet STOP_RULESET              = {1.5, 0.0, 0.0, true, true, false};
    const RuleSet BALL_PLACEMENT_RULESET    = {1.5, 0.0, 0.0, true, true, true};


};

} // ai
} // rtt

#endif //ROBOTEAM_AI_STRATEGYMANAGER_H
