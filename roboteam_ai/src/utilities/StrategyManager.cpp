//
// Created by mrlukasbos on 9-11-18.
//

#include <roboteam_ai/src/world/World.h>
#include "StrategyManager.h"

namespace rtt {
namespace ai {

void StrategyManager::setCurrentRefGameState(RefCommand command) {
    if (gameStates.find(command) != gameStates.end()) {
        this->currentRefGameState = gameStates[command];
    }
}

const RefGameState &StrategyManager::getCurrentRefGameState() const {
    return currentRefGameState;
}

} // ai
} // rtt
