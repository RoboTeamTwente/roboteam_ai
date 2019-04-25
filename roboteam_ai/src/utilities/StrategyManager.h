/*
 * Created by mrlukasbos on 9-11-18.
 *
 * Looks at the refereeData and returns the name of the desired strategy
 */

#ifndef ROBOTEAM_AI_STRATEGYMANAGER_H
#define ROBOTEAM_AI_STRATEGYMANAGER_H

#include <iostream>
#include <map>
#include "Referee.hpp"
#include "Constants.h"

namespace rtt {
namespace ai {

// used for proper parsing of the tree names
struct StrategyMap {
  RefGameState commandId;
  std::string strategyName;
  RefGameState followUpCommandId;
  StrategyMap()
          :commandId(RefGameState::UNDEFINED), strategyName(""), followUpCommandId(RefGameState::UNDEFINED) { }
  StrategyMap(RefGameState commandId, std::string strategyName,
          RefGameState followUpCommandId = RefGameState::UNDEFINED)
          :
          commandId(commandId), strategyName(strategyName), followUpCommandId(followUpCommandId) { }
};

class StrategyManager {
public:
    explicit StrategyManager() = default;
    std::string getCurrentStrategyName(roboteam_msgs::RefereeCommand currentRefCmd);
    std::string getCurrentKeeperTreeName(roboteam_msgs::RefereeCommand currentRefCmd);

private:
    StrategyMap currentStrategyMap;
    StrategyMap currentKeeperMap;
    StrategyMap getStrategyMapForRefGameState(RefGameState commandId);
    StrategyMap getKeeperMapForRefGameState(RefGameState commandId);

    std::vector<StrategyMap> strategyMaps = {
            {RefGameState::NORMAL_START, "TestStrategy"},
            {RefGameState::FORCED_START, "TestStrategy"},
            {RefGameState::HALT, "halt_strategy"},
            {RefGameState::STOP, "halt_strategy"},
            {RefGameState::TIMEOUT_US, "time_out_strategy"},
            {RefGameState::TIMEOUT_THEM, "time_out_strategy"},
            {RefGameState::GOAL_US, "kickoff_them_formation_strategy"},
            {RefGameState::GOAL_THEM, "kickoff_us_formation_strategy"},
            {RefGameState::BALL_PLACEMENT_US, "ball_placement_us_strategy"},
            {RefGameState::BALL_PLACEMENT_THEM, "ball_placement_them_strategy"},

            //  Strategies with a follow up strategy
            {RefGameState::PREPARE_KICKOFF_US, "kickoff_us_formation_strategy", RefGameState::DO_KICKOFF},
            {RefGameState::PREPARE_KICKOFF_THEM, "kickoff_them_formation_strategy", RefGameState::DEFEND_KICKOFF},
            {RefGameState::PREPARE_PENALTY_US, "prepare_penalty_us_strategy", RefGameState::DO_PENALTY},
            {RefGameState::PREPARE_PENALTY_THEM, "prepare_penalty_us_strategy", RefGameState::DEFEND_PENALTY},

            {RefGameState::DIRECT_FREE_US, "free_kick_formation_strategy"},
            {RefGameState::DIRECT_FREE_THEM, "free_kick_them_strategy"},
            {RefGameState::INDIRECT_FREE_US, "free_kick_formation_strategy"},
            {RefGameState::INDIRECT_FREE_THEM, "free_kick_them_strategy"},

            // these are called after PREPARE_
            // these custom strategies need special attention
            {RefGameState::DO_KICKOFF, "TestStrategy"},
            {RefGameState::DEFEND_KICKOFF, "TestStrategy"},
            {RefGameState::DEFEND_PENALTY, "TestStrategy"},
            {RefGameState::DO_PENALTY, "shoot_penalty_us_strategy"}
    };

    std::vector<StrategyMap> keeperMaps = {
            {RefGameState::NORMAL_START, "keeper_default_tactic"},
            {RefGameState::FORCED_START, "keeper_default_tactic"},
            {RefGameState::HALT, "keeper_halt_tactic"},
            {RefGameState::STOP, "keeper_avoid_tactic"},
            {RefGameState::TIMEOUT_US, "keeper_time_out_tactic"},
            {RefGameState::TIMEOUT_THEM, "keeper_halt_tactic"},
            {RefGameState::GOAL_US, "keeper_formation_tactic"},
            {RefGameState::GOAL_THEM, "keeper_formation_tactic"},
            {RefGameState::BALL_PLACEMENT_US, "keeper_avoid_tactic"}, // the keeper should not do ballplacement, we don't want him leaving the goal
            {RefGameState::BALL_PLACEMENT_THEM, "keeper_avoid_tactic"},

            // the keeper does not use follow up strategies
            {RefGameState::PREPARE_KICKOFF_US, "keeper_formation_tactic"},
            {RefGameState::PREPARE_KICKOFF_THEM, "keeper_formation_tactic"},
            {RefGameState::PREPARE_PENALTY_US, "keeper_formation_tactic"},
            {RefGameState::PREPARE_PENALTY_THEM, "keeper_formation_tactic"},

            {RefGameState::DIRECT_FREE_US, "keeper_default_tactic"},
            {RefGameState::DIRECT_FREE_THEM, "keeper_default_tactic"},
            {RefGameState::INDIRECT_FREE_US, "keeper_default_tactic"},
            {RefGameState::INDIRECT_FREE_THEM, "keeper_default_tactic"},

            // these are called after PREPARE_
            // these custom strategies need special attention
            {RefGameState::DO_KICKOFF, "keeper_default_tactic"},
            {RefGameState::DEFEND_KICKOFF, "keeper_default_tactic"},
            {RefGameState::DEFEND_PENALTY, "keeper_default_tactic"},
            {RefGameState::DO_PENALTY, "keeper_default_tactic"}
    };
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_STRATEGYMANAGER_H
