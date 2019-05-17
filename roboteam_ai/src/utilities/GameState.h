#ifndef ROBOTEAM_AI_GAMESTATE_H
#define ROBOTEAM_AI_GAMESTATE_H

#include <roboteam_utils/Vector2.h>

namespace rtt {
namespace ai {

struct GameState {
    GameState() { };
    GameState(std::string strategyName, std::string keeperStrategyName, std::string ruleSetName)
            : strategyName(std::move(strategyName)), keeperStrategyName(std::move(keeperStrategyName)), ruleSetName(std::move(ruleSetName))
            { };

    std::string strategyName;
    std::string keeperStrategyName;
    std::string ruleSetName;
    Vector2 ballPositionAtStartOfGameState;
    bool useKeeper = true;
    int keeperId = 0;

    RuleSet getRuleSet() {
        for (auto ruleSet : Constants::ruleSets()) {
            if (ruleSet.title == ruleSetName) {
                return ruleSet;
            }
        }
        return {};
    }
};

} // ai
} // rtt


#endif //ROBOTEAM_AI_GAMESTATE_H
