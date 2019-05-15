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
    if (command == this->currentRefGameState.getCommandId()) {
        return;
    }

    // otherwise, if we are in a followupstate and the refcommand is normal start we don't change a thing
    if (this->currentRefGameState.isFollowUpCommand() && command == RefCommand::NORMAL_START) {
        return;
    }

    // if there is a follow up command and we go into normal play, execute the follow up command instead
    // else if there is not a follow up command we can just go to the new gamestate
    if (this->currentRefGameState.hasFollowUpCommand() && command == RefCommand::NORMAL_START) {
        this->currentRefGameState = getRefGameStateForRefCommand(this->currentRefGameState.getFollowUpCommandId());
    } else {
        this->currentRefGameState = getRefGameStateForRefCommand(command);
    }
}



const RefGameState &StrategyManager::getCurrentRefGameState() const {
    return currentRefGameState;
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
