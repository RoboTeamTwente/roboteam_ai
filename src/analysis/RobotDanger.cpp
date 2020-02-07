//
// Created by mrlukasbos on 5-3-19.
//

#include "analysis/RobotDanger.h"
#include <world/FieldComputations.h>

namespace rtt::ai::analysis {

double RobotDanger::getTotalDanger(const Field &field) {
    double total = 0.0;
    if (hasBall) total += 25;
    if (goalVisionPercentage > 20) total += 15;
    if (distanceToGoal < field.getFieldWidth() / 3) total += 20;
    if (shortestDistToEnemy > 0.5) total += 15;
    if (aimedAtGoal) total += 10;
    if (closingInToGoal) total += 5;
    if (!robotsToPassTo.empty()) total += 10;
    return total;
}

}  // namespace rtt::ai::analysis