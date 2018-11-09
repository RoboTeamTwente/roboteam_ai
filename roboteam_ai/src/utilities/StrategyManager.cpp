//
// Created by mrlukasbos on 9-11-18.
//

#include "StrategyManager.h"

namespace rtt {
namespace ai {

std::string StrategyManager::getCurrentStrategyName() {
    roboteam_msgs::RefereeCommand currentRefCmd = Referee::getRefereeData().command;

    RefGameState commandFromMostRecentReferee = static_cast<RefGameState>(currentRefCmd.command);
    StrategyMap strategy = strategyMaps.at(currentRefCmd.command);
    StrategyMap nextStrategy;

    // if the command has a followUpCommand and the ref says normalPlay we need to run the followupcommand
    if (currentStrategyMap.followUpCommandId != RefGameState::UNDEFINED
            && commandFromMostRecentReferee == RefGameState::NORMAL_START) {
        nextStrategy = strategyMaps.at(static_cast<unsigned long>(currentStrategyMap.followUpCommandId));
    } else {
        nextStrategy = strategyMaps.at(currentRefCmd.command);
    }

    currentStrategyMap = nextStrategy;
    return nextStrategy.strategyName;
}

} // ai
} // rtt