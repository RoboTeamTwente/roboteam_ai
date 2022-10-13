#ifndef ROBOTEAM_AI_GAMESTATE_H
#define ROBOTEAM_AI_GAMESTATE_H

#include <roboteam_utils/Vector2.h>

#include "Constants.h"
#include "RuleSet.h"

namespace rtt::ai {

struct GameState {
    GameState() = default;
    GameState(std::string strategyName, std::string ruleSetName) : ruleSetName(std::move(ruleSetName)), strategyName(std::move(strategyName)){};

    std::string ruleSetName;
    int keeperId = Constants::DEFAULT_KEEPER_ID();

    RuleSet getRuleSet() {
        for (auto ruleSet : Constants::ruleSets()) {
            if (ruleSet.title == ruleSetName) {
                return ruleSet;
            }
        }
        std::cerr << "Returning empty ruleset with name '" << ruleSetName << "', this should never happen!" << std::endl;
        return {};
    }

    std::string getStrategyName() { return strategyName; }

   private:
    std::string strategyName;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_GAMESTATE_H
