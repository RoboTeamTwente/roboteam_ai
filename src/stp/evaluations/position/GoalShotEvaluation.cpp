//
// Created by maxl on 23-03-21.
//

#include "include/roboteam_ai/stp/evaluations/position/GoalShotEvaluation.h"
#include <cmath>

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
         * -> Linear from 0% to 70%, after 70% max score.
         */
        double evalScore = std::max(std::min(0.7,vis)/0.7,0.0)
                    /// Distance, Linear-Factor -> Favor closer
                    //Closer is better, caps at far distances -> Max score till 2 meters, linear from 2 to 7 meters.
                 * std::max(std::min(5.0,(7-dist))/5,0.0)
                    ///Angle, Linear -> Favor in front
                    // In front in better -> Max score till 30 degrees, linear from 30 to 90.
                  * std::max(std::min(M_PI_2-angle,M_PI/6)/M_PI/6,0.0);
        return calculateMetric(evalScore);
    }

    uint8_t GoalShotEvaluation::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }
}