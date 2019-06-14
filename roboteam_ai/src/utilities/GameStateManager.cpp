#include <roboteam_ai/src/interface/api/Output.h>
#include "GameStateManager.hpp"

namespace rtt {
namespace ai {

roboteam_msgs::RefereeData GameStateManager::refMsg;
StrategyManager GameStateManager::strategymanager;

roboteam_msgs::RefereeData GameStateManager::getRefereeData() {
    return GameStateManager::refMsg;
}

void GameStateManager::setRefereeData(roboteam_msgs::RefereeData refMsg) {
    GameStateManager::refMsg = refMsg;
    auto cmd = static_cast<RefCommand>(refMsg.command.command);
    //if we have shootouts, we need different handling
    if (static_cast<int>(refMsg.stage.stage) == roboteam_msgs::RefereeStage::PENALTY_SHOOTOUT) {
        if (cmd == RefCommand::PREPARE_PENALTY_US) {
            strategymanager.setCurrentRefGameState(RefCommand::PREPARE_SHOOTOUT_US);
        }
        else if (cmd == RefCommand::PREPARE_PENALTY_THEM) {
            strategymanager.setCurrentRefGameState(RefCommand::PREPARE_SHOOTOUT_THEM);
        }
        else {
            strategymanager.setCurrentRefGameState(cmd);
        }
        return;
    }
    strategymanager.setCurrentRefGameState(cmd);
}

// Initialize static variables
GameState GameStateManager::getCurrentGameState() {
    GameState newGameState;
    if (interface::Output::usesRefereeCommands()) {
        newGameState = static_cast<GameState>(strategymanager.getCurrentRefGameState());
        newGameState.keeperId = getRefereeData().us.goalie;
        // if there is a ref we set the interface gamestate to these values as well
        // this makes sure that when we stop using the referee we don't return to an unknown state,
        // // so now we keep the same.
        interface::Output::setInterfaceGameState(newGameState);
    }
    else {
        newGameState = interface::Output::getInterfaceGameState();
    }
    return newGameState;
}

void GameStateManager::forceNewGameState(RefCommand cmd) {
    std::cout << "Forcing new refstate!" << std::endl;

    // overwrite both the interface and the strategy manager.
    interface::Output::setInterfaceGameState(strategymanager.getRefGameStateForRefCommand(cmd));
    strategymanager.forceCurrentRefGameState(cmd);
}

bool GameStateManager::canEnterDefenseArea(int robotId) {
    GameState currentState = getCurrentGameState();
    if (robotId != currentState.keeperId) {
        return currentState.getRuleSet().robotsCanEnterDefenseArea();
    }
    return true;
}

bool GameStateManager::canMoveOutsideField(int robotId) {
    GameState currentState = getCurrentGameState();
    if (robotId != currentState.keeperId) {
        return currentState.getRuleSet().robotsCanGoOutOfField;
    }
    return true;
}
}//ai
}//rtt
