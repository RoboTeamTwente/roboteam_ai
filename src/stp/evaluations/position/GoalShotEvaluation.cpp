//
// Created by maxl on 23-03-21.
//

#include "include/roboteam_ai/stp/evaluations/position/GoalShotEvaluation.h"
#include <math.h>
#include <iostream>

namespace rtt::ai::stp::evaluation {
    GoalShotEvaluation::GoalShotEvaluation() noexcept {
        /**
         * Creates a piecewise linear function that looks as follows:
         *
         * (0,255)  |        XXXXXX
         *          |      XXX
         *          |    XXX
         *          |  XXX
         *   (0,0)  |XXX
         *                 (evalScore)
         */
        piecewiseLinearFunction = nativeformat::param::createParam(control_constants::FUZZY_FALSE, control_constants::FUZZY_TRUE, control_constants::FUZZY_FALSE, "positionGoalShot");
        piecewiseLinearFunction->setYAtX(control_constants::FUZZY_FALSE, 0.0);
        piecewiseLinearFunction->linearRampToYAtX(control_constants::FUZZY_TRUE, 1);
        piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, 1);
    }

    uint8_t GoalShotEvaluation::metricCheck(double vis, double dist, double angle) const noexcept {
        /**
         * Visibility, Root-Function -> Favor Higher visibility
         * Too low vis means not a valid pos, difference between higher vis doesnt matter as much (not linear), high vis is caped.
         */
        double evalScore = std::max(std::min(0.7,vis)/0.7,0.0)
                /**
                 * Distance, Linear-Factor -> Favor closer
                 * Closer is better, caps at far distances
                 */
                 * std::max(std::min(5.0,(7-dist))/5,0.0)
                 /**
                  * Angle, Linear -> Favor in front
                  * In front in better
                  */
                  * std::max(std::min(M_PI_2-angle,M_PI_4)/M_PI_4,0.0);
        return calculateMetric(evalScore);
    }

    uint8_t GoalShotEvaluation::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }
}