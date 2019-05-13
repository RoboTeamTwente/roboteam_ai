//
// Created by mrlukasbos on 9-11-18.
//

#include <roboteam_ai/src/world/World.h>
#include "StrategyManager.h"

namespace rtt {
namespace ai {

StrategyMap StrategyManager::getCurrentStrategy(roboteam_msgs::RefereeCommand currentRefCmd) {
    auto commandFromMostRecentReferee = static_cast<RefCommand>(currentRefCmd.command);
    // trigger only if the command changed
    if (commandFromMostRecentReferee != prevCmd) {
        auto oldStrategyMap = currentStrategyMap;

        // if the command has a followUpCommand and the ref says normalPlay we need to run the followupcommand
        if (oldStrategyMap.followUpCommandId != RefCommand::UNDEFINED
            && commandFromMostRecentReferee == RefCommand::NORMAL_START) {
            return getStrategyMapForRefGameState(oldStrategyMap.followUpCommandId);
        }

        currentStrategyMap = getStrategyMapForRefGameState(commandFromMostRecentReferee);
        prevCmd = commandFromMostRecentReferee;
    }
    return currentStrategyMap;
}

/// Use an iterator and a lambda to efficiently get the Node for a specified id
StrategyMap StrategyManager::getStrategyMapForRefGameState(RefCommand commandId) {
    return *std::find_if(strategyMaps.begin(), strategyMaps.end(), [commandId](StrategyMap map) {
      return map.commandId == commandId;
    });
}

std::string StrategyManager::getCurrentKeeperTreeName(roboteam_msgs::RefereeCommand currentRefCmd) {
    auto commandFromMostRecentReferee = static_cast<RefCommand>(currentRefCmd.command);

    // HACK the keeper now always follows the same 'custom ref command' as the normal strategy
    currentKeeperMap = getKeeperMapForRefGameState(currentStrategyMap.commandId);
    return currentKeeperMap.strategyName;
}

StrategyMap StrategyManager::getKeeperMapForRefGameState(RefCommand commandId) {
    return *std::find_if(keeperMaps.begin(), keeperMaps.end(), [commandId](StrategyMap map) {
        return map.commandId == commandId;
    });
}

} // ai
} // rtt
