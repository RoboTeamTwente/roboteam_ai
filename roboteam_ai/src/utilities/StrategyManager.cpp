//
// Created by mrlukasbos on 9-11-18.
//

#include <roboteam_ai/src/world/World.h>
#include "StrategyManager.h"

namespace rtt {
namespace ai {

// process ref commands
void StrategyManager::setCurrentRefGameState(RefCommand command) {

    // if the command is the same, we don't need to do anything
    if (command == GameStateManager::getCurrentRefGameState().getCommandId()) {
        return;
    }

    // otherwise, if we are in a followupstate and the refcommand is normal start we don't change a thing
    if (GameStateManager::getCurrentRefGameState().isFollowUpCommand() && command == RefCommand::NORMAL_START) {
        return;
    }


    // we need to change refgamestate here
    RefGameState newState;
    if (GameStateManager::getCurrentRefGameState().hasFollowUpCommand() && command == RefCommand::NORMAL_START) {
        newState = getRefGameStateForRefCommand(GameStateManager::getCurrentRefGameState().getFollowUpCommandId());
    } else {
        newState = getRefGameStateForRefCommand(command);
    }
    newState.setBallPositionAtStartOfRefGameState(world::world->getBall()->pos);
    GameStateManager::setCurrentRefGameState(newState);
}



RefGameState StrategyManager::getCurrentRefGameState() {
    return GameStateManager::getCurrentRefGameState();
}

const RefGameState StrategyManager::getRefGameStateForRefCommand(RefCommand command) {
    for (auto gameState : this->gameStates) {
        if (gameState.getCommandId() == command) {
            return gameState;
        }
    }
    std::cerr << "Returning an undefined refstate! This should never happen!" << std::endl;
    return gameStates[0];
}

} // ai
} // rtt
