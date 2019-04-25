//
// Created by mrlukasbos on 9-11-18.
//

#include <roboteam_ai/src/world/World.h>
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

        // hack to switch refstate
        if (world::world->getBall()->vel.length() > 1.0) {
            currentStrategyMap = nextStrategy;
        }
        // end hack

        nextStrategy = getStrategyMapForRefGameState(currentStrategyMap.followUpCommandId);
    }
    else {
        currentStrategyMap = nextStrategy;
        nextStrategy = getStrategyMapForRefGameState(commandFromMostRecentReferee);
    }

    return nextStrategy.strategyName;
}

/// Use an iterator and a lambda to efficiently get the Node for a specified id
StrategyMap StrategyManager::getStrategyMapForRefGameState(RefGameState commandId) {
    return *std::find_if(strategyMaps.begin(), strategyMaps.end(), [commandId](StrategyMap map) {
      return map.commandId == commandId;
    });
}

std::string StrategyManager::getCurrentKeeperTreeName(roboteam_msgs::RefereeCommand currentRefCmd) {
    auto commandFromMostRecentReferee = static_cast<RefGameState>(currentRefCmd.command);
    currentKeeperMap = getKeeperMapForRefGameState(commandFromMostRecentReferee);
    return currentKeeperMap.strategyName;
}

StrategyMap StrategyManager::getKeeperMapForRefGameState(RefGameState commandId) {
    return *std::find_if(keeperMaps.begin(), keeperMaps.end(), [commandId](StrategyMap map) {
        return map.commandId == commandId;
    });
}

} // ai
} // rtt