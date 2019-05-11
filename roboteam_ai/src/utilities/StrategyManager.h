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
  RefCommand commandId;
  std::string strategyName;
  double maxVel;
  RefCommand followUpCommandId;

  StrategyMap() : commandId(RefCommand::UNDEFINED), strategyName(""), maxVel(rtt::ai::Constants::MAX_VEL()), followUpCommandId(RefCommand::UNDEFINED) { }

  StrategyMap(RefCommand commandId, std::string strategyName,
          double maxVel = rtt::ai::Constants::MAX_VEL(),
          RefCommand followUpCommandId = RefCommand::UNDEFINED)
          : commandId(commandId), strategyName(strategyName), maxVel(maxVel), followUpCommandId(followUpCommandId) { }
};

class StrategyManager {
public:
    explicit StrategyManager() = default;
    StrategyMap getCurrentStrategy(roboteam_msgs::RefereeCommand currentRefCmd);
    std::string getCurrentKeeperTreeName(roboteam_msgs::RefereeCommand currentRefCmd);

private:
    RefCommand prevCmd = RefCommand::HALT;
    StrategyMap currentStrategyMap = {RefCommand::HALT, "halt_strategy"};
    StrategyMap currentKeeperMap = {RefCommand::HALT, "keeper_halt_tactic"};
    StrategyMap getStrategyMapForRefGameState(RefCommand commandId);
    StrategyMap getKeeperMapForRefGameState(RefCommand commandId);

    std::vector<StrategyMap> strategyMaps = {
            {RefCommand::NORMAL_START, "normal_play_strategy"},
            {RefCommand::FORCED_START, "normal_play_strategy"},
            {RefCommand::HALT, "halt_strategy", 0.0},
            {RefCommand::STOP, "stop_strategy", Constants::MAX_STOP_STATE_VEL()},
            {RefCommand::TIMEOUT_US, "time_out_strategy"},
            {RefCommand::TIMEOUT_THEM, "time_out_strategy"},
            {RefCommand::GOAL_US, "kickoff_them_formation_strategy"},
            {RefCommand::GOAL_THEM, "kickoff_us_formation_strategy"},
            {RefCommand::BALL_PLACEMENT_US, "ball_placement_us_strategy", Constants::MAX_VEL_BALLPLACEMENT()},
            {RefCommand::BALL_PLACEMENT_THEM, "ball_placement_them_strategy", Constants::MAX_VEL_BALLPLACEMENT()},

            //  Strategies with a follow up strategy
            {RefCommand::PREPARE_KICKOFF_US, "kickoff_us_formation_strategy", rtt::ai::Constants::MAX_VEL(), RefCommand::DO_KICKOFF},
            {RefCommand::PREPARE_PENALTY_US, "penalty_us_prepare_strategy", rtt::ai::Constants::MAX_VEL(), RefCommand::DO_PENALTY},

            {RefCommand::PREPARE_KICKOFF_THEM, "kickoff_them_formation_strategy",  rtt::ai::Constants::MAX_VEL(), RefCommand::DEFEND_KICKOFF},
            {RefCommand::PREPARE_PENALTY_THEM, "penalty_them_strategy", rtt::ai::Constants::MAX_VEL(), RefCommand::DEFEND_PENALTY},

            {RefCommand::DIRECT_FREE_US, "free_kick_shoot_strategy"},
            {RefCommand::DIRECT_FREE_THEM, "free_kick_them_strategy"},
            {RefCommand::INDIRECT_FREE_US, "free_kick_shoot_strategy"},
            {RefCommand::INDIRECT_FREE_THEM, "free_kick_them_strategy"},

            // these are called after PREPARE_
            // these custom strategies need special attention
            {RefCommand::DO_KICKOFF, "kickoff_shoot_strategy"},
            {RefCommand::DEFEND_KICKOFF, "kickoff_them_strategy"},
            {RefCommand::DEFEND_PENALTY, "penalty_them_strategy"},
            {RefCommand::DO_PENALTY, "penalty_us_shoot_strategy"}
    };

    std::vector<StrategyMap> keeperMaps = {
            {RefCommand::NORMAL_START, "keeper_default_tactic"},
            {RefCommand::FORCED_START, "keeper_default_tactic"},
            {RefCommand::HALT, "keeper_halt_tactic"},
            {RefCommand::STOP, "keeper_avoid_tactic"},
            {RefCommand::TIMEOUT_US, "keeper_time_out_tactic"},
            {RefCommand::TIMEOUT_THEM, "keeper_halt_tactic"},
            {RefCommand::GOAL_US, "keeper_formation_tactic"},
            {RefCommand::GOAL_THEM, "keeper_formation_tactic"},
            {RefCommand::BALL_PLACEMENT_US, "keeper_avoid_tactic"}, // the keeper should not do ballplacement, we don't want him leaving the goal
            {RefCommand::BALL_PLACEMENT_THEM, "keeper_avoid_tactic"},

            // the keeper does not use follow up strategies
            {RefCommand::PREPARE_KICKOFF_US, "keeper_formation_tactic"},
            {RefCommand::PREPARE_KICKOFF_THEM, "keeper_default_tactic"},
            {RefCommand::PREPARE_PENALTY_US, "keeper_formation_tactic"},
            {RefCommand::PREPARE_PENALTY_THEM, "keeper_penalty_tactic"},

            {RefCommand::DIRECT_FREE_US, "keeper_default_tactic"},
            {RefCommand::DIRECT_FREE_THEM, "keeper_default_tactic"},
            {RefCommand::INDIRECT_FREE_US, "keeper_default_tactic"},
            {RefCommand::INDIRECT_FREE_THEM, "keeper_default_tactic"},

            // these are called after PREPARE_
            // these custom strategies need special attention
            {RefCommand::DO_KICKOFF, "keeper_default_tactic"},
            {RefCommand::DEFEND_KICKOFF, "keeper_default_tactic"},
            {RefCommand::DEFEND_PENALTY, "keeper_penalty_tactic"},
            {RefCommand::DO_PENALTY, "keeper_default_tactic"}
    };
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_STRATEGYMANAGER_H
