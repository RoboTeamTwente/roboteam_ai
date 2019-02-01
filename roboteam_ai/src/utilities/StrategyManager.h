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
    private:
        StrategyMap currentStrategyMap;
        StrategyMap getStrategyMapForRefGameState(RefGameState commandId);

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
                {RefGameState::PREPARE_PENALTY_US, "BallPlacementUsStrategy", RefGameState::DEFEND_PENALTY},
                {RefGameState::PREPARE_PENALTY_THEM, "BallPlacementThemStrategy", RefGameState::DO_PENALTY},

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
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_STRATEGYMANAGER_H
