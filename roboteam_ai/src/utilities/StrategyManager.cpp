//
// Created by mrlukasbos on 9-11-18.
//

#include "../world/World.h"
#include "../world/Ball.h"
#include "StrategyManager.h"
#include "GameStateManager.hpp"

namespace rtt {
namespace ai {

// process ref commands
void StrategyManager::setCurrentRefGameState(RefCommand command, roboteam_msgs::RefereeStage stage) {
    // if the stage is shootout, we interpret penalty commands as shootOut penalty commands
    if (stage.stage == roboteam_msgs::RefereeStage::PENALTY_SHOOTOUT) {
        if (command == RefCommand::PREPARE_PENALTY_US) {
            command = RefCommand::PREPARE_SHOOTOUT_US;
        }
        else if (command == RefCommand::PREPARE_PENALTY_THEM) {
            command = RefCommand::PREPARE_SHOOTOUT_THEM;
        }
    }

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
    }
    else {
        newState = getRefGameStateForRefCommand(command);
    }
    if (world::world->getBall()) {
        newState.ballPositionAtStartOfGameState = world::world->getBall()->pos;
    }
    else {
        newState.ballPositionAtStartOfGameState = {0, 0};
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

void StrategyManager::forceCurrentRefGameState(RefCommand command) {
    // we need to change refgamestate here
    RefGameState newState = getRefGameStateForRefCommand(command);
    if (world::world->getBall()) {
        newState.ballPositionAtStartOfGameState = world::world->getBall()->pos;
    }
    else {
        newState.ballPositionAtStartOfGameState = {0, 0};
    }
    currentRefGameState = newState;
}

} // ai
} // rtt
