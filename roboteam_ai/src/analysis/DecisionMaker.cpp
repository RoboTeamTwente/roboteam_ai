//
// Created by mrlukasbos on 9-4-19.
//

#include "DecisionMaker.h"
#include "GameAnalyzer.h"


namespace rtt {
namespace ai {
namespace analysis {

PlayStyle DecisionMaker::getRecommendedPlayStyle() {

  AnalysisReport report = * GameAnalyzer::getInstance().getMostRecentReport();
  BallPossession possession = report.ballPossession;


  int amountOfRobots = world::world->getUs().size();

  PlayStyle styles[9][3]  = {
          {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // 0
          {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, // 1
          {{1, 0, 0}, {1, 0, 0}, {0, 1, 0}}, // 2
          {{2, 0, 0}, {1, 1, 0}, {1, 1, 0}}, // 3
          {{3, 0, 0}, {2, 1, 0}, {1, 1, 1}}, // 4
          {{3, 1, 0}, {2, 2, 0}, {2, 1, 1}}, // 5
          {{4, 1, 0}, {2, 2, 1}, {2, 1, 2}}, // 6
          {{4, 2, 0}, {3, 2, 1}, {2, 2, 2}}, // 7
          {{5, 2, 0}, {3, 2, 2}, {2, 2, 3}}  // 8
  };

  return styles[amountOfRobots][possession];
}


} // analysis
} // ai
} // rtt