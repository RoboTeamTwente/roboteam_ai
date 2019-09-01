#include <include/roboteam_ai/Settings/Settings.h>
#include "include/roboteam_ai/interface/api/Output.h"
#include "include/roboteam_ai/utilities/GameStateManager.hpp"

namespace rtt {
namespace ai {

roboteam_proto::SSL_Referee GameStateManager::refMsg;
StrategyManager GameStateManager::strategymanager;
    std::mutex GameStateManager::refMsgLock;

roboteam_proto::SSL_Referee GameStateManager::getRefereeData() {
    std::lock_guard<std::mutex> lock(refMsgLock);
    return GameStateManager::refMsg;
}

void GameStateManager::setRefereeData(roboteam_proto::SSL_Referee refMsg) {
    std::lock_guard<std::mutex> lock(refMsgLock);
    GameStateManager::refMsg = refMsg;
    auto cmd = static_cast<RefCommand>(refMsg.command());
    auto stage = refMsg.stage();
    strategymanager.setCurrentRefGameState(cmd,stage);
}

// Initialize static variables
GameState GameStateManager::getCurrentGameState() {
    GameState newGameState;
    if (interface::Output::usesRefereeCommands()) {
        newGameState = static_cast<GameState>(strategymanager.getCurrentRefGameState());

        if (SETTINGS.isYellow()) {
            newGameState.keeperId = getRefereeData().yellow().goalie();
        } else {
            newGameState.keeperId = getRefereeData().blue().goalie();
        }
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

    Vector2 GameStateManager::getRefereeDesignatedPosition() {
        auto designatedPos = rtt::ai::GameStateManager::getRefereeData().designated_position();
        return Vector2(designatedPos.x()/1000, designatedPos.x()/1000);
    }
}//ai
}//rtt
