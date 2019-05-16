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
    RefGameState getCurrentRefGameState();
    void setCurrentRefGameState(RefCommand command);
    const RefGameState getRefGameStateForRefCommand(RefCommand command);

private:
    const std::vector<RefGameState> gameStates = {

         // failsafe: for an undefined refstate everything should halt
         RefGameState(RefCommand::UNDEFINED,            "halt_strategy",                   "keeper_halt_tactic",      "halt"),

         RefGameState(RefCommand::NORMAL_START,         "normal_play_strategy",            "keeper_default_tactic",   "default"),
         RefGameState(RefCommand::FORCED_START,         "normal_play_strategy",            "keeper_default_tactic",   "default"),
         RefGameState(RefCommand::HALT,                 "halt_strategy",                   "keeper_halt_tactic",      "halt"),
         RefGameState(RefCommand::STOP,                 "stop_strategy",                   "keeper_avoid_tactic",     "stop"),
         RefGameState(RefCommand::TIMEOUT_US,           "time_out_strategy",               "keeper_time_out_tactic",  "default"),
         RefGameState(RefCommand::TIMEOUT_THEM,         "halt_strategy",                   "keeper_halt_tactic",      "default"),
         RefGameState(RefCommand::GOAL_US,              "kickoff_them_formation_strategy", "keeper_formation_tactic", "default"),
         RefGameState(RefCommand::GOAL_THEM,            "kickoff_us_formation_strategy",   "keeper_formation_tactic", "default"),
         RefGameState(RefCommand::BALL_PLACEMENT_US,    "ball_placement_us_strategy",      "keeper_avoid_tactic",     "ballplacement"),
         RefGameState(RefCommand::BALL_PLACEMENT_THEM,  "kickoff_them_formation_strategy", "keeper_avoid_tactic",     "ballplacement"),
         RefGameState(RefCommand::DIRECT_FREE_US,       "free_kick_shoot_strategy",        "keeper_default_tactic",   "default"),
         RefGameState(RefCommand::DIRECT_FREE_THEM,     "free_kick_them_strategy",         "keeper_default_tactic",   "default"),
         RefGameState(RefCommand::INDIRECT_FREE_US,     "free_kick_shoot_strategy",        "keeper_default_tactic",   "default"),
         RefGameState(RefCommand::INDIRECT_FREE_THEM,   "free_kick_them_strategy",         "keeper_default_tactic",   "default"),

         // prepare commands
         // These have a follow up command
         RefGameState(RefCommand::PREPARE_KICKOFF_US,   "kickoff_us_formation_strategy",   "keeper_formation_tactic", "default",  false, RefCommand::DO_KICKOFF),
         RefGameState(RefCommand::PREPARE_KICKOFF_THEM, "kickoff_them_formation_strategy", "keeper_default_tactic",   "default",  false, RefCommand::DEFEND_KICKOFF),
         RefGameState(RefCommand::PREPARE_PENALTY_US,   "penalty_us_prepare_strategy",     "keeper_formation_tactic", "default",  false, RefCommand::DO_PENALTY),
         RefGameState(RefCommand::PREPARE_PENALTY_THEM, "penalty_them_strategy",           "keeper_penalty_tactic",   "default",  false, RefCommand::DEFEND_PENALTY),

         // follow up commands
         // these are custom commands, called when 'normal play' is called after a prepare_ command
         RefGameState(RefCommand::DO_KICKOFF,           "kickoff_shoot_strategy",          "keeper_default_tactic",   "default", true),
         RefGameState(RefCommand::DEFEND_KICKOFF,       "kickoff_them_strategy",           "keeper_default_tactic",   "default", true),
         RefGameState(RefCommand::DO_PENALTY,           "penalty_us_shoot_strategy",       "keeper_default_tactic",   "default", true),
         RefGameState(RefCommand::DEFEND_PENALTY,       "penalty_them_strategy",           "keeper_penalty_tactic",   "default", true)
    };
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_STRATEGYMANAGER_H
