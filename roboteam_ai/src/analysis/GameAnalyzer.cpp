//
// Created by mrlukasbos on 19-2-19.
//

#include "GameAnalyzer.h"
#include "../utilities/World.h"
#include "../utilities/Field.h"

namespace rtt {
namespace ai {
namespace analysis {

GameAnalyzer::GameAnalyzer() { }

double GameAnalyzer::getBallPossessionEstimate(bool ourTeam) {
    double ballPossessionEstimate = 0;

    // booleans that matter
    bool teamHasBall = World::teamHasBall(ourTeam);
    bool teamHasShotAtGoal = true;
    bool teamHasAttackerShotAtGoal = true;

    // doubles that matter
    double teamDistanceToGoal = 0;
    return 0;
}

double GameAnalyzer::getBallSecurityForTeam(bool ourTeam) {

    return 0;
}

} // analysis
} // ai
} // rtt