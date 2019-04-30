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
  double maxVel;
  RefGameState followUpCommandId;

  StrategyMap() : commandId(RefGameState::UNDEFINED), strategyName(""), maxVel(rtt::ai::Constants::MAX_VEL()), followUpCommandId(RefGameState::UNDEFINED) { }

  StrategyMap(RefGameState commandId, std::string strategyName,
          double maxVel = rtt::ai::Constants::MAX_VEL(),
          RefGameState followUpCommandId = RefGameState::UNDEFINED)
          : commandId(commandId), strategyName(strategyName), maxVel(maxVel), followUpCommandId(followUpCommandId) { }
};

class StrategyManager {
public:
    explicit StrategyManager() = default;
    StrategyMap getCurrentStrategy(roboteam_msgs::RefereeCommand currentRefCmd);
    std::string getCurrentKeeperTreeName(roboteam_msgs::RefereeCommand currentRefCmd);

private:
    RefGameState prevCmd = RefGameState::HALT;
    StrategyMap currentStrategyMap = {RefGameState::HALT, "halt_strategy"};
    StrategyMap currentKeeperMap = {RefGameState::HALT, "keeper_halt_tactic"};
    StrategyMap getStrategyMapForRefGameState(RefGameState commandId);
    StrategyMap getKeeperMapForRefGameState(RefGameState commandId);

    std::vector<StrategyMap> strategyMaps = {
            {RefGameState::NORMAL_START, "normal_play_strategy"},
            {RefGameState::FORCED_START, "normal_play_strategy"},
            {RefGameState::HALT, "halt_strategy", 0.0},
            {RefGameState::STOP, "stop_strategy", Constants::MAX_STOP_STATE_VEL()},
            {RefGameState::TIMEOUT_US, "time_out_strategy"},
            {RefGameState::TIMEOUT_THEM, "time_out_strategy"},
            {RefGameState::GOAL_US, "kickoff_them_formation_strategy"},
            {RefGameState::GOAL_THEM, "kickoff_us_formation_strategy"},
            {RefGameState::BALL_PLACEMENT_US, "ball_placement_us_strategy", Constants::MAX_VEL_BALLPLACEMENT()},
            {RefGameState::BALL_PLACEMENT_THEM, "ball_placement_them_strategy", Constants::MAX_VEL_BALLPLACEMENT()},

            //  Strategies with a follow up strategy
            {RefGameState::PREPARE_KICKOFF_US, "kickoff_us_formation_strategy", rtt::ai::Constants::MAX_VEL(), RefGameState::DO_KICKOFF},
            {RefGameState::PREPARE_PENALTY_US, "penalty_us_prepare_strategy", rtt::ai::Constants::MAX_VEL(), RefGameState::DO_PENALTY},

            {RefGameState::PREPARE_KICKOFF_THEM, "kickoff_them_formation_strategy",  rtt::ai::Constants::MAX_VEL(), RefGameState::DEFEND_KICKOFF},
            {RefGameState::PREPARE_PENALTY_THEM, "penalty_them_strategy"},

            {RefGameState::DIRECT_FREE_US, "free_kick_shoot_strategy"},
            {RefGameState::DIRECT_FREE_THEM, "free_kick_them_strategy"},
            {RefGameState::INDIRECT_FREE_US, "free_kick_shoot_strategy"},
            {RefGameState::INDIRECT_FREE_THEM, "free_kick_them_strategy"},

            // these are called after PREPARE_
            // these custom strategies need special attention
            {RefGameState::DO_KICKOFF, "kickoff_shoot_strategy"},
            {RefGameState::DEFEND_KICKOFF, "kickoff_them_strategy"},
            {RefGameState::DEFEND_PENALTY, "penalty_them_strategy"},
            {RefGameState::DO_PENALTY, "penalty_us_shoot_strategy"}
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
            {RefGameState::DEFEND_PENALTY, "keeper_penalty_tactic"},
            {RefGameState::DO_PENALTY, "keeper_default_tactic"}
    };
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_STRATEGYMANAGER_H
