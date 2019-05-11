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
    explicit StrategyManager() = default;
    const RefGameState &getCurrentRefGameState() const;
    void setCurrentRefGameState(RefCommand command);

private:
    RefGameState currentRefGameState;

    // Basic rulesets for rule compliance
    const RuleSet DEFAULT_RULESET           = {8.0, 1.5, 6.5, false, true, true};
    const RuleSet HALT_RULESET              = {0.0, 0.0, 0.0, true, true, true};
    const RuleSet STOP_RULESET              = {1.5, 0.0, 0.0, true, true, false};
    const RuleSet BALL_PLACEMENT_RULESET    = {1.5, 0.0, 0.0, true, true, true};

    std::map<RefCommand, RefGameState> gameStates = {
        {RefCommand::NORMAL_START,          RefGameState("normal_play_strategy",               "keeper_default_tactic",        DEFAULT_RULESET)},
        {RefCommand::FORCED_START,          RefGameState("normal_play_strategy",               "keeper_default_tactic",        DEFAULT_RULESET)},
        {RefCommand::HALT,                  RefGameState("halt_strategy",                      "keeper_halt_tactic",           HALT_RULESET)},
        {RefCommand::STOP,                  RefGameState("stop_strategy",                      "keeper_avoid_tactic",          STOP_RULESET)},
        {RefCommand::TIMEOUT_US,            RefGameState("time_out_strategy",                  "keeper_time_out_tactic",       DEFAULT_RULESET)},
        {RefCommand::TIMEOUT_THEM,          RefGameState("halt_strategy",                      "keeper_halt_tactic",           DEFAULT_RULESET)},
        {RefCommand::GOAL_US,               RefGameState("kickoff_them_formation_strategy",    "keeper_formation_tactic",      DEFAULT_RULESET)},
        {RefCommand::GOAL_THEM,             RefGameState("kickoff_us_formation_strategy",      "keeper_formation_tactic",      DEFAULT_RULESET)},
        {RefCommand::BALL_PLACEMENT_US,     RefGameState("ball_placement_us_strategy",         "keeper_avoid_tactic",          BALL_PLACEMENT_RULESET)},
        {RefCommand::BALL_PLACEMENT_THEM,   RefGameState("kickoff_them_formation_strategy",    "keeper_avoid_tactic",          BALL_PLACEMENT_RULESET)},
        {RefCommand::PREPARE_KICKOFF_US,    RefGameState("kickoff_us_formation_strategy",      "keeper_formation_tactic",      DEFAULT_RULESET)},
        {RefCommand::PREPARE_KICKOFF_THEM,  RefGameState("kickoff_them_formation_strategy",    "keeper_default_tactic",        DEFAULT_RULESET)},
        {RefCommand::PREPARE_PENALTY_US,    RefGameState("penalty_us_prepare_strategy",        "keeper_formation_tactic",      DEFAULT_RULESET)},
        {RefCommand::PREPARE_PENALTY_THEM,  RefGameState("penalty_them_strategy",              "keeper_penalty_tactic",        DEFAULT_RULESET)},
        {RefCommand::DIRECT_FREE_US,        RefGameState("free_kick_shoot_strategy",           "keeper_default_tactic",        DEFAULT_RULESET)},
        {RefCommand::DIRECT_FREE_THEM,      RefGameState("free_kick_them_strategy",            "keeper_default_tactic",        DEFAULT_RULESET)},
        {RefCommand::INDIRECT_FREE_US,      RefGameState("free_kick_shoot_strategy",           "keeper_default_tactic",        DEFAULT_RULESET)},
        {RefCommand::INDIRECT_FREE_THEM,    RefGameState("free_kick_them_strategy",            "keeper_default_tactic",        DEFAULT_RULESET)},
        {RefCommand::DO_KICKOFF,            RefGameState("kickoff_shoot_strategy",             "keeper_default_tactic",        DEFAULT_RULESET)},
        {RefCommand::DEFEND_KICKOFF,        RefGameState("kickoff_them_strategy",              "keeper_default_tactic",        DEFAULT_RULESET)},
        {RefCommand::DEFEND_PENALTY,        RefGameState("penalty_them_strategy",              "keeper_penalty_tactic",        DEFAULT_RULESET)},
        {RefCommand::DO_PENALTY,            RefGameState("penalty_us_shoot_strategy",           "keeper_default_tactic",       DEFAULT_RULESET)}
    };
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_STRATEGYMANAGER_H
