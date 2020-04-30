//
// Created by jesse on 27-03-20.
//

#include <roboteam_utils/Print.h>
#include <roboteam_utils/Vector2.h>
#include <include/roboteam_ai/world_new/World.hpp>
#include "stp/new_plays_analysis/PassProblem.h"
using namespace pagmo;
namespace rtt::ai::stp{
    vector_double PassProblem::fitness(const vector_double &dv) const {
        auto score = 0.0;
        auto point = Vector2(dv[0], dv[1]);
        RTT_DEBUG("HI IM HAPPENING")
        score = cost_function(point, problemWorld);

        return {score};
    }


    std::pair<vector_double, vector_double> PassProblem::get_bounds() const {
        // Field bounds, adjusted for robot radius (so the robot always stays fully inside the field at all times)
        auto xboundright = problemField.getRightmostX() - stp::control_constants::ROBOT_RADIUS;
        auto xboundleft = problemField.getLeftmostX() + stp::control_constants::ROBOT_RADIUS;
        auto yboundbottom = problemField.getBottommostY() + stp::control_constants::ROBOT_RADIUS;
        auto yboundtop = problemField.getTopmostY() - stp::control_constants::ROBOT_RADIUS;

        return {{xboundleft, yboundbottom}, {xboundright, yboundtop}};
    }


    void PassProblem::updateInfoForProblem(world_new::view::WorldDataView problemWorld, world::Field field) {
        //std::lock_guard<std::mutex> guard(world_mutex);
        this->problemWorld = problemWorld;
        this->problemField = field;
    }

    double PassProblem::shootSuccesReward(Vector2 point, world_new::view::WorldDataView world) const {
        auto w = world_new::World::instance()->getWorld().value();
        double percentage = FieldComputations::getPercentageOfGoalVisibleFromPoint(problemField, false, point, w, -1, true);
        return percentage;
    }

    double PassProblem::cost_function(const Vector2 &point, world_new::view::WorldDataView world) const {
        auto score = 0.0;

        if (ai::FieldComputations::pointIsInDefenseArea(problemField, point)) {
            score = 500;
            return {score};
        }

        auto theirClosestBot = world.getRobotClosestToPoint(point, world_new::Team::us);
        auto theirClosestDistance = (theirClosestBot->getPos() - point).length();

        auto ourClosestBot = world.getRobotClosestToPoint(point, world_new::Team::them);
        auto ourClosestDistance = (ourClosestBot->getPos() - point).length();
        score += -100 * theirClosestDistance;
        score += 100 * ourClosestDistance;

        score += -10 * shootSuccesReward(point, world);
        score += -ai::FieldComputations::getDistanceToGoal(problemField, false, point);

        return {score};
    }



}