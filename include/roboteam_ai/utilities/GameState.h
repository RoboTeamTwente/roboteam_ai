#ifndef ROBOTEAM_AI_GAMESTATE_H
#define ROBOTEAM_AI_GAMESTATE_H

#include <roboteam_utils/Vector2.h>
#include "RuleSet.h"
#include "Constants.h"

namespace rtt {
namespace ai {

struct GameState {
    /**
     * Why is this class defined here, and not in its own
     * translation unit?
     */
    GameState() = default;
    GameState(std::string strategyName, std::string keeperStrategyName, std::string ruleSetName)
            : strategyName(std::move(strategyName)), keeperStrategyName(std::move(keeperStrategyName)), ruleSetName(std::move(ruleSetName))
            { };

    std::string strategyName;
    std::string keeperStrategyName;
    std::string ruleSetName;
    Vector2 ballPositionAtStartOfGameState;
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
};

} // ai
} // rtt


#endif //ROBOTEAM_AI_GAMESTATE_H
