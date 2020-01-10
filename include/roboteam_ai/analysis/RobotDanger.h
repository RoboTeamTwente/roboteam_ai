//
// Created by mrlukasbos on 5-3-19.
//

#ifndef ROBOTEAM_AI_ROBOTDANGER_H
#define ROBOTEAM_AI_ROBOTDANGER_H

#include "world/Field.h"

namespace rtt {
namespace ai {
namespace analysis {

struct RobotDanger {
    bool ourTeam;
    int id;

    double shortestDistToEnemy;
    double distanceToGoal;
    bool aimedAtGoal;
    bool closingInToGoal;
    bool hasBall;
    double goalVisionPercentage;
    std::vector<std::pair<int, double>> robotsToPassTo;
    double getTotalDanger(const Field &field);
};

} // analysis
} // ai
} // rtt


#endif //ROBOTEAM_AI_ROBOTDANGER_H
