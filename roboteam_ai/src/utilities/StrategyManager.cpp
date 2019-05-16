//
// Created by mrlukasbos on 9-11-18.
//

#include <roboteam_ai/src/world/World.h>
#include "StrategyManager.h"
#include "GameStateManager.hpp"

namespace rtt {
namespace ai {

// process ref commands
void StrategyManager::setCurrentRefGameState(RefCommand command) {

    // if the command is the same, we don't need to do anything
    if (command == currentRefGameState.commandId) {
        return;
    }

    // otherwise, if we are in a followupstate and the refcommand is normal start we don't change a thing
    if (currentRefGameState.isfollowUpCommand && command == RefCommand::NORMAL_START) {
        return;
    }

    // we need to change refgamestate here
    RefGameState newState;
    if (currentRefGameState.hasFollowUpCommand() && command == RefCommand::NORMAL_START) {
        newState = getRefGameStateForRefCommand(currentRefGameState.followUpCommandId);
    } else {
        newState = getRefGameStateForRefCommand(command);
    }
    if (world::world->getBall()) {
        newState.ballPositionAtStartOfGameState = world::world->getBall()->pos;
    } else {
        newState.ballPositionAtStartOfGameState = {0,0};
    }
    currentRefGameState = newState;
}



RefGameState StrategyManager::getCurrentRefGameState() {
    return currentRefGameState;
}

const RefGameState StrategyManager::getRefGameStateForRefCommand(RefCommand command) {
    for (auto gameState : this->gameStates) {
        if (gameState.commandId == command) {
            return gameState;
        }
    }
    std::cerr << "Returning an undefined refstate! This should never happen!" << std::endl;
    return gameStates[0];
}

} // ai
} // rtt
