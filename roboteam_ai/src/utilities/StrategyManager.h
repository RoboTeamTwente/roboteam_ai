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
                {RefGameState::NORMAL_START, "bigjson/NormalPlay"},
                {RefGameState::FORCED_START, "bigjson/009472f6-0d76-4db6-8161-a536bf497f89"},
                {RefGameState::HALT, "rtt_dennis/HaltStrategy"},
                {RefGameState::STOP, "rtt_anouk/StopStrat"},
                {RefGameState::TIMEOUT_US, "rtt_anouk/StopStrat"},
                {RefGameState::TIMEOUT_THEM, "rtt_anouk/StopStrat"},
                {RefGameState::GOAL_US, "rtt_anouk/StopStrat"},
                {RefGameState::GOAL_THEM, "rtt_anouk/StopStrat"},
                {RefGameState::BALL_PLACEMENT_US, "rtt_anouk/BallPlacement_Strat"},
                {RefGameState::BALL_PLACEMENT_THEM, "rtt_anouk/BallPlacementThemStrat"},

                //  Strategies with a follow up strategy
                {RefGameState::PREPARE_KICKOFF_US, "rtt_emiel/PrepareKickoffUsStrategy", RefGameState::DO_KICKOFF},
                {RefGameState::PREPARE_KICKOFF_THEM, "rtt_emiel/PrepareKickoffThemStrategy",
                 RefGameState::DEFEND_KICKOFF},
                {RefGameState::PREPARE_PENALTY_US, "rtt_emiel/PreparePenaltyUsStrategy", RefGameState::DEFEND_PENALTY},
                {RefGameState::PREPARE_PENALTY_THEM, "rtt_emiel/PreparePenaltyThemStrategy", RefGameState::DO_PENALTY},

                {RefGameState::DIRECT_FREE_US, "rtt_jim/NormalPlay"},
                {RefGameState::DIRECT_FREE_THEM, "rtt_anouk/PrepareDirectThem"},
                {RefGameState::INDIRECT_FREE_US, "rtt_emiel/IndirectUsStrategy"},
                {RefGameState::INDIRECT_FREE_THEM, "rtt_anouk/PrepareDirectThem"},

                // these are called after PREPARE_
                // these custom strategies need special attention
                {RefGameState::DO_KICKOFF, "rtt_bob/KickoffWithChipStrategy"},
                {RefGameState::DEFEND_KICKOFF, "rtt_jim/KickOffDefenseStrat"},
                {RefGameState::DEFEND_PENALTY, "rtt_emiel/PreparePenaltyThemStrategy"},
                {RefGameState::DO_PENALTY, "rtt_jim/TakePenalty"}
        };
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_STRATEGYMANAGER_H
