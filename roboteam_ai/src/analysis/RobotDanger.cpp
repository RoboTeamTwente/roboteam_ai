//
// Created by mrlukasbos on 5-3-19.
//

#include <roboteam_ai/src/world/Field.h>
#include "RobotDanger.h"

namespace rtt {
namespace ai {
namespace analysis {

double RobotDanger::getTotalDanger() {
    double total = 0.0;
    if (hasBall) total += 25;
    if (goalVisionPercentage > 20) total += 15;
    if (distanceToGoal < world::field->get_field().field_width/3) total += 20;
    if (shortestDistToEnemy > 0.5) total += 15;
    if (aimedAtGoal) total += 10;
    if (closingInToGoal) total += 5;
    if (!robotsToPassTo.empty()) total += 10;
    return total;
}

} // analysis
} // ai
} // rtt