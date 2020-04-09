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
            score += 300;
        }

        auto theirClosestBot = problemWorld->getWorld()->getRobotClosestToPoint(point, world_new::Team::us);
        auto theirClosestDistance = (theirClosestBot->getPos() - point).length();

        auto ourClosestBot = problemWorld->getWorld()->getRobotClosestToPoint(point, world_new::Team::them);
        auto ourClosestDistance = (ourClosestBot->getPos() - point).length();
        score += -100 * theirClosestDistance;
        score += 100 * ourClosestDistance;
        return {score};
    }

//        score = 0
//        if main_field.in_defense_area(xpoint, ypoint):
//        score += 300
//
//        bot, dist = world.their_closest_robot_to_point(xpoint, ypoint)
//        score += -100 * dist
//
//        ourbot, ourdist = world.our_closest_robot_to_point(xpoint, ypoint)
//        score += 100 * ourdist
//
//        shoot_succes_reward = bot.shoot_from_pos(xpoint, ypoint)
//        score += -shoot_succes_reward * 2
//        score += field.distance_to_enemy_goal(field, xpoint, ypoint)
//        return [score]




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


}
