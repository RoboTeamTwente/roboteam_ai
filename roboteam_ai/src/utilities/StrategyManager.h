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
using namespace std::string_literals;

struct StrategyMap {
  RefGameState commandId;
  std::string strategyName;
  RefGameState followUpCommandId = RefGameState::UNDEFINED;
};

class StrategyManager {
public:
    explicit StrategyManager() = default;
    std::string getCurrentStrategyName(roboteam_msgs::RefereeCommand currentRefCmd);
private:
    StrategyMap currentStrategyMap;
    StrategyMap getStrategyMapForRefGameState(RefGameState commandId);
    const std::vector<StrategyMap> strategyMaps = {
        {RefGameState::NORMAL_START,         "bigjson/NormalPlay"s},
        {RefGameState::FORCED_START,         "bigjson/009472f6-0d76-4db6-8161-a536bf497f89"s},
        {RefGameState::HALT,                 "rtt_dennis/HaltStrategy"s},
        {RefGameState::STOP,                 "rtt_anouk/StopStrat"s},
        {RefGameState::TIMEOUT_US,           "rtt_anouk/StopStrat"s},
        {RefGameState::TIMEOUT_THEM,         "rtt_anouk/StopStrat"s},
        {RefGameState::GOAL_US,              "rtt_anouk/StopStrat"s},
        {RefGameState::GOAL_THEM,            "rtt_anouk/StopStrat"s},
        {RefGameState::BALL_PLACEMENT_US,    "rtt_anouk/BallPlacement_Strat"s},
        {RefGameState::BALL_PLACEMENT_THEM,  "rtt_anouk/BallPlacementThemStrat"s},

        //  Strategies with a follow up strategy
        {RefGameState::PREPARE_KICKOFF_US,   "rtt_emiel/PrepareKickoffUsStrategy"s, RefGameState::DO_KICKOFF},
        {RefGameState::PREPARE_KICKOFF_THEM, "rtt_emiel/PrepareKickoffThemStrategy"s, RefGameState::DEFEND_KICKOFF},
        {RefGameState::PREPARE_PENALTY_US,   "rtt_emiel/PreparePenaltyUsStrategy"s, RefGameState::DEFEND_PENALTY},
        {RefGameState::PREPARE_PENALTY_THEM, "rtt_emiel/PreparePenaltyThemStrategy"s, RefGameState::DO_PENALTY},

        {RefGameState::DIRECT_FREE_US,       "rtt_jim/NormalPlay"s},
        {RefGameState::DIRECT_FREE_THEM,     "rtt_anouk/PrepareDirectThem"s},
        {RefGameState::INDIRECT_FREE_US,     "rtt_emiel/IndirectUsStrategy"s},
        {RefGameState::INDIRECT_FREE_THEM,   "rtt_anouk/PrepareDirectThem"s},

        // these are called after PREPARE_
        // these custom strategies need special attention
        {RefGameState::DO_KICKOFF,           "rtt_bob/KickoffWithChipStrategy"s},
        {RefGameState::DEFEND_KICKOFF,       "rtt_jim/KickOffDefenseStrat"s},
        {RefGameState::DEFEND_PENALTY,       "rtt_emiel/PreparePenaltyThemStrategy"s},
        {RefGameState::DO_PENALTY,           "rtt_jim/TakePenalty"s},
    };
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_STRATEGYMANAGER_H
