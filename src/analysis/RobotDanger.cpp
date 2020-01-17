//
// Created by mrlukasbos on 5-3-19.
//

#include "analysis/RobotDanger.h"
#include <world/Field.h>

namespace rtt::ai::analysis {

double RobotDanger::getTotalDanger() {
    double total = 0.0;
    if (hasBall) total += 25;
    if (distanceToGoal < world::field->get_field().get(FIELD_WIDTH) / 3) total += 20;
    if (shortestDistToEnemy > 0.5) total += 15;
    if (aimedAtGoal) total += 10;
    if (closingInToGoal) total += 5;
    if (!robotsToPassTo.empty()) total += 10;
    return total;
}

}  // namespace rtt::ai::analysis