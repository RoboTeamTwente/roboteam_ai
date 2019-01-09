//
// Created by mrlukasbos on 9-11-18.
//

#include "StrategyManager.h"

namespace rtt {
namespace ai {

std::string StrategyManager::getCurrentStrategyName(roboteam_msgs::RefereeCommand currentRefCmd) {

    auto commandFromMostRecentReferee = static_cast<RefGameState>(currentRefCmd.command);
    StrategyMap strategy = getStrategyMapForRefGameState(commandFromMostRecentReferee);
    StrategyMap nextStrategy;

    // if the command has a followUpCommand and the ref says normalPlay we need to run the followupcommand
    if (currentStrategyMap.followUpCommandId != RefGameState::UNDEFINED
            && commandFromMostRecentReferee == RefGameState::NORMAL_START) {
        nextStrategy = getStrategyMapForRefGameState(currentStrategyMap.followUpCommandId);
    }
    else {
        nextStrategy = getStrategyMapForRefGameState(commandFromMostRecentReferee);
    }

    currentStrategyMap = nextStrategy;
    return nextStrategy.strategyName;
}

/// Use an iterator and a lambda to efficiently get the Node for a specified id
StrategyMap StrategyManager::getStrategyMapForRefGameState(RefGameState commandId) {
    return *std::find_if(strategyMaps.begin(), strategyMaps.end(), [commandId](StrategyMap map) {
      return map.commandId == commandId;
    });
}

} // ai
} // rtt