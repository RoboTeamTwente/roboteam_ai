//
// Created by rolf on 23-10-18.
//

#include "GameStateManager.hpp"

namespace rtt {
namespace ai {

// Initialize static variables
roboteam_msgs::RefereeData GameStateManager::refMsg;
roboteam_msgs::RefereeData GameStateManager::previousRefMsg;
RefGameState GameStateManager::currentRefGameState;
RuleSet GameStateManager::ruleset;

roboteam_msgs::RefereeData GameStateManager::getRefereeData() {
    return GameStateManager::refMsg;
}

void GameStateManager::setRefereeData(roboteam_msgs::RefereeData refMsg) {
    GameStateManager::previousRefMsg = GameStateManager::refMsg; // set the old message as previous message
    GameStateManager::refMsg = refMsg;
}

roboteam_msgs::RefereeData GameStateManager::getPreviousRefereeData() {
    return GameStateManager::previousRefMsg;
}

RefGameState GameStateManager::getCurrentRefGameState() {
    return currentRefGameState;
}

void GameStateManager::setCurrentRefGameState(RefGameState currentRefGameState) {
    GameStateManager::currentRefGameState = currentRefGameState;
    setCurrentRuleSet(currentRefGameState.getRuleSet().title);
}

void GameStateManager::setCurrentRuleSet(std::string ruleset) {
    GameStateManager::ruleset = getRuleSetByName(ruleset);
}

RuleSet GameStateManager::getRuleSetByName(std::string name) {

    for (auto ruleSet : Constants::ruleSets()) {
        if (ruleSet.title == name) {
            return ruleSet;
        }
    }
    return {};
}


const RuleSet &GameStateManager::getRuleset() {
    return ruleset;
}

}//ai
}//rtt
