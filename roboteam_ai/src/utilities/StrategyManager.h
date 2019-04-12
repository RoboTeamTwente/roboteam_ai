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
            {RefGameState::NORMAL_START, "twoPlayerStrategyV2"},
            {RefGameState::FORCED_START, "threePlayerStrategyV2"},
            {RefGameState::HALT, "haltStrategy"},
            {RefGameState::STOP, "haltStrategy"},
            {RefGameState::TIMEOUT_US, "haltStrategy"},
            {RefGameState::TIMEOUT_THEM, "haltStrategy"},
            {RefGameState::GOAL_US, "haltStrategy"},
            {RefGameState::GOAL_THEM, "haltStrategy"},
            {RefGameState::BALL_PLACEMENT_US, "BallPlacementUsStrategy"},
            {RefGameState::BALL_PLACEMENT_THEM, "BallPlacementThemStrategy"},

            //  Strategies with a follow up strategy
            {RefGameState::PREPARE_KICKOFF_US, "EnterFormationStrategy", RefGameState::DO_KICKOFF},
            {RefGameState::PREPARE_KICKOFF_THEM, "EnterFormationStrategy", RefGameState::DEFEND_KICKOFF},
            {RefGameState::PREPARE_PENALTY_US, "BallPlacementUsStrategy", RefGameState::DO_PENALTY},
            {RefGameState::PREPARE_PENALTY_THEM, "BallPlacementThemStrategy", RefGameState::DEFEND_PENALTY},

            {RefGameState::DIRECT_FREE_US, "twoPlayerStrategyV2"},
            {RefGameState::DIRECT_FREE_THEM, "EnterFormationStrategy"},
            {RefGameState::INDIRECT_FREE_US, "EnterFormationStrategy"},
            {RefGameState::INDIRECT_FREE_THEM, "EnterFormationStrategy"},

            // these are called after PREPARE_
            // these custom strategies need special attention
            {RefGameState::DO_KICKOFF, "twoPlayerStrategyV2"},
            {RefGameState::DEFEND_KICKOFF, "twoPlayerStrategyV2"},
            {RefGameState::DEFEND_PENALTY, "EnterFormationStrategy"},
            {RefGameState::DO_PENALTY, "EnterFormationStrategy"}
    };

    std::vector<StrategyMap> keeperMaps = {
            {RefGameState::NORMAL_START, "SingleKeeperTactic"},
            {RefGameState::FORCED_START, "SingleKeeperTactic"},
            {RefGameState::HALT, "haltTactic"},
            {RefGameState::STOP, "haltTactic"},
            {RefGameState::TIMEOUT_US, "haltTactic"},
            {RefGameState::TIMEOUT_THEM, "haltTactic"},
            {RefGameState::GOAL_US, "SingleKeeperTactic"},
            {RefGameState::GOAL_THEM, "SingleKeeperTactic"},
            {RefGameState::BALL_PLACEMENT_US, "AvoidBallTactic"}, // the keeper should not do ballplacement, we don't want him leaving the goal
            {RefGameState::BALL_PLACEMENT_THEM, "AvoidBallTactic"},

            // the keeper does not use follow up strategies
            {RefGameState::PREPARE_KICKOFF_US, "SingleKeeperTactic"},
            {RefGameState::PREPARE_KICKOFF_THEM, "SingleKeeperTactic"},
            {RefGameState::PREPARE_PENALTY_US, "SingleKeeperTactic"},
            {RefGameState::PREPARE_PENALTY_THEM, "SingleKeeperTactic"},

            {RefGameState::DIRECT_FREE_US, "SingleKeeperTactic"},
            {RefGameState::DIRECT_FREE_THEM, "SingleKeeperTactic"},
            {RefGameState::INDIRECT_FREE_US, "SingleKeeperTactic"},
            {RefGameState::INDIRECT_FREE_THEM, "SingleKeeperTactic"},

            // these are called after PREPARE_
            // these custom strategies need special attention
            {RefGameState::DO_KICKOFF, "SingleKeeperTactic"},
            {RefGameState::DEFEND_KICKOFF, "SingleKeeperTactic"},
            {RefGameState::DEFEND_PENALTY, "SingleKeeperTactic"},
            {RefGameState::DO_PENALTY, "SingleKeeperTactic"}
    };
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_STRATEGYMANAGER_H
