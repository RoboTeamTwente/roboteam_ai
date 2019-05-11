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

    // if we use the keeper we should check if it is currently visible
    // if it is not visible we should claim all robots visible in world
    // otherwise we should subtract the keeper
    bool keeperIsInWorld = std::find_if(world::world->getUs().begin(), world::world->getUs().end(), [](world::Robot robot) {
        return robot.id == robotDealer::RobotDealer::getKeeperID();
    }) != world::world->getUs().end();

    // subtract one robot if we need a keeper
    if (robotDealer::RobotDealer::usesSeparateKeeper() && keeperIsInWorld) {
        amountOfRobots = std::max(0, amountOfRobots-1);
    }

  PlayStyle styles[9][5]  = {

          {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // 0
          {{1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 1, 0}}, // 1
          {{2, 0, 0}, {2, 0, 0}, {1, 1, 0}, {1, 1, 0}, {1, 0, 1}}, // 2
          {{3, 0, 0}, {2, 1, 0}, {2, 1, 0}, {1, 1, 1}, {1, 1, 1}}, // 3
          {{3, 1, 0}, {3, 1, 0}, {2, 2, 0}, {2, 1, 1}, {2, 1, 1}}, // 4
          {{4, 1, 0}, {4, 1, 0}, {2, 2, 1}, {2, 1, 2}, {2, 0, 3}}, // 5
          {{4, 2, 0}, {4, 2, 0}, {3, 1, 2}, {2, 2, 2}, {2, 1, 3}}, // 6
          {{5, 2, 0}, {5, 1, 1}, {4, 2, 1}, {3, 3, 1}, {3, 1, 3}}, // 7 // Normal
          {{5, 3, 0}, {5, 2, 1}, {4, 2, 2}, {3, 3, 2}, {3, 1, 4}}  // 8
  };



    return styles[amountOfRobots][possession];
}


} // analysis
} // ai
} // rtt