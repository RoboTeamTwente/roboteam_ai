#include "DecisionMaker.h"

#include "roboteam_ai/src/utilities/RobotDealer.h"
#include "GameAnalyzer.h"

namespace rtt {
namespace ai {
namespace analysis {

PlayStyle DecisionMaker::getRecommendedPlayStyle(BallPossession possession) {
    int amountOfRobots = world::world->getUs().size();

    // subtract one robot if we have a keeper
    if (robotDealer::RobotDealer::keeperExistsInWorld()) {
        amountOfRobots = std::max(0, amountOfRobots - 1);
    }

    PlayStyle playStyle[9][5] = {
            {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // 0
            {{1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {0, 0, 1}, {0, 0, 1}}, // 1
            {{2, 0, 0}, {1, 0, 1}, {1, 0, 1}, {1, 0, 1}, {1, 0, 1}}, // 2
            {{3, 0, 0}, {2, 0, 1}, {2, 0, 1}, {2, 0, 1}, {2, 0, 1}}, // 3
            {{4, 0, 0}, {3, 1, 0}, {3, 1, 0}, {3, 1, 0}, {2, 1, 1}}, // 4
            {{4, 1, 0}, {4, 1, 0}, {4, 1, 0}, {3, 1, 1}, {2, 1, 2}}, // 5
            {{5, 1, 0}, {5, 1, 0}, {4, 1, 1}, {3, 1, 2}, {2, 1, 3}}, // 6
            {{5, 1, 1}, {4, 2, 1}, {4, 2, 1}, {3, 2, 2}, {2, 2, 3}}, // 7 // Normal
            {{6, 1, 0}, {5, 2, 1}, {5, 2, 1}, {4, 2, 2}, {3, 2, 3}}  // 8
    };

    return playStyle[amountOfRobots][possession];
}

} // analysis
} // ai
} // rtt