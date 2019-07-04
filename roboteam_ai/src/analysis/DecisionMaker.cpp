//
// Created by mrlukasbos on 9-4-19.
//

#include <roboteam_ai/src/utilities/RobotDealer.h>
#include "DecisionMaker.h"
#include "GameAnalyzer.h"


namespace rtt {
namespace ai {
namespace analysis {

PlayStyle DecisionMaker::getRecommendedPlayStyle(BallPossession possession) {
    int amountOfRobots = world::world->getUs().size();

    // subtract one robot if we have a keeper
    if (robotDealer::RobotDealer::keeperExistsInWorld()) {
        amountOfRobots = std::max(0, amountOfRobots-1);
    }
  PlayStyle styles[9][5]  = {

          {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // 0
          {{1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {0, 0, 1}, {0, 0, 1}}, // 1
          {{2, 0, 0}, {1, 0, 1}, {1, 0, 1}, {1, 0, 1}, {1, 0, 1}}, // 2
          {{3, 0, 0}, {2, 0, 1}, {2, 0, 1}, {2, 0, 1}, {2, 0, 1}}, // 3
          {{4, 0, 0}, {3, 0, 1}, {3, 0, 1}, {3, 0, 1}, {2, 0, 2}}, // 4
          {{4, 0, 1}, {4, 0, 1}, {4, 0, 1}, {3, 0, 2}, {3, 0, 2}}, // 5
          {{5, 0, 1}, {5, 0, 1}, {4, 0, 2}, {3, 0, 3}, {3, 0, 3}}, // 6
          {{5, 1, 1}, {4, 1, 2}, {4, 1, 2}, {3, 1, 3}, {3, 0, 4}}, // 7 // Normal
          {{6, 0, 2}, {5, 0, 3}, {5, 0, 3}, {4, 0, 4}, {4, 0, 4}}  // 8
  };



    return styles[amountOfRobots][possession];
}


} // analysis
} // ai
} // rtt