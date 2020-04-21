//
// Created by jesse on 27-03-20.
//

#include <roboteam_utils/Print.h>
#include <roboteam_utils/Vector2.h>
#include <include/roboteam_ai/world_new/World.hpp>
#include "include/roboteam_ai/stp/new_plays_analysis/PassProblem.h"
using namespace pagmo;
namespace rtt::ai::stp{
    vector_double PassProblem::fitness(const vector_double &dv) const {
        auto score = 0.0;
        auto point = Vector2(dv[0], dv[1]);

        if (ai::FieldComputations::pointIsInDefenceArea(problemWorld->getField().value(), point)) {
            score = 500;
            return {score};
        }

        auto theirClosestBot = problemWorld->getWorld()->getRobotClosestToPoint(point, world_new::Team::us);
        auto theirClosestDistance = (theirClosestBot->getPos() - point).length();

        auto ourClosestBot = problemWorld->getWorld()->getRobotClosestToPoint(point, world_new::Team::them);
        auto ourClosestDistance = (ourClosestBot->getPos() - point).length();
        score += -100 * theirClosestDistance;
        score += 100 * ourClosestDistance;

        score += -100 * shootSuccesReward(Vector2());
        score += -ai::FieldComputations::getDistanceToGoal(problemWorld->getField().value(), false, point);

        return {score};
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
            return (x-min)/(max-min);
        }
        else {
            return 0;
        }
    }

    void PassProblem::updateInfoForProblem(world_new::World* problemWorld) {
        this->problemWorld = problemWorld;
    }

    double PassProblem::shootSuccesReward(Vector2 point) const {
        auto w = world_new::World::instance()->getWorld().value();
        auto p = Vector2(0,0);
        double percentage = FieldComputations::getPercentageOfGoalVisibleFromPoint(problemWorld->getField().value(), true, p, w, -1, true);
        if(percentage != 100) {
            RTT_DEBUG(percentage)

        }
        return percentage;
    }


}
