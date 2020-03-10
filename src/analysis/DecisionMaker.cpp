//
// Created by mrlukasbos on 9-4-19.
//

#include "analysis/DecisionMaker.h"

#include <utilities/RobotDealer.h>

#include "analysis/GameAnalyzer.h"

namespace rtt::ai::analysis {

PlayStyle DecisionMaker::getRecommendedPlayStyle(BallPossession possession, uint8_t amountOfRobots) {
    // subtract one robot if we have a keeper
    if (robotDealer::RobotDealer::keeperExistsInWorld()) {
        amountOfRobots = std::max(0, amountOfRobots - 1);
    }
    // static -> prevent instantiation more than once, gets placed in static memory
    static PlayStyle styles[9][5] = {
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},  // 0
        {{1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {0, 0, 1}, {0, 0, 1}},  // 1
        {{2, 0, 0}, {1, 0, 1}, {1, 0, 1}, {1, 0, 1}, {1, 0, 1}},  // 2
        {{3, 0, 0}, {2, 0, 1}, {2, 0, 1}, {2, 0, 1}, {2, 0, 1}},  // 3
        {{4, 0, 0}, {3, 1, 0}, {3, 1, 0}, {3, 1, 0}, {2, 1, 1}},  // 4
        {{4, 1, 0}, {4, 1, 0}, {4, 1, 0}, {3, 1, 1}, {2, 1, 2}},  // 5
        {{5, 1, 0}, {5, 1, 0}, {4, 1, 1}, {3, 1, 2}, {2, 1, 3}},  // 6
        {{5, 1, 1}, {4, 2, 1}, {4, 2, 1}, {3, 2, 2}, {3, 1, 3}},  // 7 // Normal
        {{6, 0, 2}, {5, 0, 3}, {5, 0, 3}, {4, 0, 4}, {4, 1, 3}}   // 8
    };

    return styles[amountOfRobots][possession];
}

}  // namespace rtt::ai::analysis