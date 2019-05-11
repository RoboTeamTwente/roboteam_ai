//
// Created by mrlukasbos on 9-11-18.
//

#include <roboteam_ai/src/world/World.h>
#include "StrategyManager.h"

namespace rtt {
namespace ai {

void StrategyManager::setCurrentRefGameState(RefCommand command) {
    if (gameStates.find(command) != gameStates.end()) {
        this->currentRefGameState = gameStates[command];
    }
}

const RefGameState &StrategyManager::getCurrentRefGameState() const {
    return currentRefGameState;
}

StrategyManager::StrategyManager() {
     this->gameStates = {
        std::make_pair(RefCommand::NORMAL_START,          RefGameState("normal_play_strategy",               "keeper_default_tactic",        DEFAULT_RULESET)),
        std::make_pair(RefCommand::FORCED_START,          RefGameState("normal_play_strategy",               "keeper_default_tactic",        DEFAULT_RULESET)),
        std::make_pair(RefCommand::HALT,                  RefGameState("halt_strategy",                      "keeper_halt_tactic",           HALT_RULESET)),
        std::make_pair(RefCommand::STOP,                  RefGameState("stop_strategy",                      "keeper_avoid_tactic",          STOP_RULESET)),
        std::make_pair(RefCommand::TIMEOUT_US,            RefGameState("time_out_strategy",                  "keeper_time_out_tactic",       DEFAULT_RULESET)),
        std::make_pair(RefCommand::TIMEOUT_THEM,          RefGameState("halt_strategy",                      "keeper_halt_tactic",           DEFAULT_RULESET)),
        std::make_pair(RefCommand::GOAL_US,               RefGameState("kickoff_them_formation_strategy",    "keeper_formation_tactic",      DEFAULT_RULESET)),
        std::make_pair(RefCommand::GOAL_THEM,             RefGameState("kickoff_us_formation_strategy",      "keeper_formation_tactic",      DEFAULT_RULESET)),
        std::make_pair(RefCommand::BALL_PLACEMENT_US,     RefGameState("ball_placement_us_strategy",         "keeper_avoid_tactic",          BALL_PLACEMENT_RULESET)),
        std::make_pair(RefCommand::BALL_PLACEMENT_THEM,   RefGameState("kickoff_them_formation_strategy",    "keeper_avoid_tactic",          BALL_PLACEMENT_RULESET)),
        std::make_pair(RefCommand::PREPARE_KICKOFF_US,    RefGameState("kickoff_us_formation_strategy",      "keeper_formation_tactic",      DEFAULT_RULESET)),
        std::make_pair(RefCommand::PREPARE_KICKOFF_THEM,  RefGameState("kickoff_them_formation_strategy",    "keeper_default_tactic",        DEFAULT_RULESET)),
        std::make_pair(RefCommand::PREPARE_PENALTY_US,    RefGameState("penalty_us_prepare_strategy",        "keeper_formation_tactic",      DEFAULT_RULESET)),
        std::make_pair(RefCommand::PREPARE_PENALTY_THEM,  RefGameState("penalty_them_strategy",              "keeper_penalty_tactic",        DEFAULT_RULESET)),
        std::make_pair(RefCommand::DIRECT_FREE_US,        RefGameState("free_kick_shoot_strategy",           "keeper_default_tactic",        DEFAULT_RULESET)),
        std::make_pair(RefCommand::DIRECT_FREE_THEM,      RefGameState("free_kick_them_strategy",            "keeper_default_tactic",        DEFAULT_RULESET)),
        std::make_pair(RefCommand::INDIRECT_FREE_US,      RefGameState("free_kick_shoot_strategy",           "keeper_default_tactic",        DEFAULT_RULESET)),
        std::make_pair(RefCommand::INDIRECT_FREE_THEM,    RefGameState("free_kick_them_strategy",            "keeper_default_tactic",        DEFAULT_RULESET)),
        std::make_pair(RefCommand::DO_KICKOFF,            RefGameState("kickoff_shoot_strategy",             "keeper_default_tactic",        DEFAULT_RULESET)),
        std::make_pair(RefCommand::DEFEND_KICKOFF,        RefGameState("kickoff_them_strategy",              "keeper_default_tactic",        DEFAULT_RULESET)),
        std::make_pair(RefCommand::DEFEND_PENALTY,        RefGameState("penalty_them_strategy",              "keeper_penalty_tactic",        DEFAULT_RULESET)),
        std::make_pair(RefCommand::DO_PENALTY,            RefGameState("penalty_us_shoot_strategy",           "keeper_default_tactic",       DEFAULT_RULESET))
    };
     this->currentRefGameState = gameStates[RefCommand::HALT];
}

} // ai
} // rtt
