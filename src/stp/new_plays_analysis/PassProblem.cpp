//
// Created by timo on 4/8/20.
//

#include "stp/new_plays_analysis/PassProblem.h"

#include <roboteam_utils/Print.h>
#include <roboteam_utils/Vector2.h>

using namespace pagmo;
namespace rtt::ai::stp {
vector_double PassProblem::fitness(const vector_double& dv) const {
    // Expectation for passpoint
    // p(successfully_get_there)*reward + p(fail)*risk_if_pass_gets_intercepted
    // reward = max {
    //                  shoot_reward
    //                  next_pass_reward
    //              }
    auto field = problemWorld->getField();
    double p_success = 0.5;
    double reward;
    auto point = Vector2(dv[0], dv[1]);
    auto dist_to_goal = (field->getTheirGoalCenter() - point).length();
    bool inTheirDefenseArea = false;  // ai::FieldComputations::pointIsInDefenceArea(field.value(), point, false);
    reward = dist_to_goal;
    auto robot = problemWorld->getWorld()->getRobotClosestToPoint(point, world_new::Team::them);
    if (inTheirDefenseArea) {
        reward = reward - 100;
    }

    reward -= (robot->getPos() - point).length();

    double risk = 9;
    double expectation = p_success * reward + (1 - p_success) * 0;

    return {expectation};
}

std::pair<vector_double, vector_double> PassProblem::get_bounds() const {
    // Field bounds, adjusted for robot radius (so the robot always stays fully inside the field at all times)
    auto xboundright = problemWorld->getField()->getRightmostX() - stp::control_constants::ROBOT_RADIUS;
    auto xboundleft = problemWorld->getField()->getLeftmostX() + stp::control_constants::ROBOT_RADIUS;
    auto yboundbottom = problemWorld->getField()->getBottommostY() + stp::control_constants::ROBOT_RADIUS;
    auto yboundtop = problemWorld->getField()->getTopmostY() - stp::control_constants::ROBOT_RADIUS;

    return {{xboundleft, yboundbottom}, {xboundright, yboundtop}};
}

const double PassProblem::rel(double x, double min, double max) const {
    if (x >= max) {
        return 1;
    }
    if (min <= x <= max) {
        return (x - min) / (max - min);
    } else {
        return 0;
    }
}

void PassProblem::updateInfoForProblem(world_new::World* problemWorld) { this->problemWorld = problemWorld; }

}  // namespace rtt::ai::stp
