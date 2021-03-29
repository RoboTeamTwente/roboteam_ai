//
// Created by maxl on 24-03-21.
//

#include "include/roboteam_ai/stp/evaluations/position/TimeToPositionEvaluation.h"

namespace rtt::ai::stp::evaluation {
    TimeToPositionEvaluation::TimeToPositionEvaluation() noexcept {
        /**
         * Creates a piecewise linear function that looks as follows:
         *
         * (0,255)  |           XXXX
         *          |         XX
         *          |       XX
         *          |     XX
         *   (0,0)  |XXXXX
         *          ---|0.8|1|1.2|--
         *                 (Ratio)
         */
        piecewiseLinearFunction = nativeformat::param::createParam(control_constants::FUZZY_FALSE, control_constants::FUZZY_TRUE, control_constants::FUZZY_FALSE, "PositionTimeCompareToPoint");
        piecewiseLinearFunction->setYAtX(control_constants::FUZZY_FALSE, 0.8);
        piecewiseLinearFunction->linearRampToYAtX(control_constants::FUZZY_TRUE, 1.2);
        piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, 1.2);
    }

    uint8_t TimeToPositionEvaluation::metricCheck(std::optional<world::view::RobotView> robot1, std::optional<world::view::RobotView> robot2, Vector2 point) const noexcept {
        if (!robot1.has_value()) return 0;
        if (!robot2.has_value()) return 255;
        //TODO when new path-planning is implemented use integrated functions, for now stupid, later path-planning
        double r1TimeEst = (point-robot1.value()->getPos()).length();
        double r2TimeEst = (point-robot2.value()->getPos()).length();
        return calculateMetric(r2TimeEst/r1TimeEst);
    }

    uint8_t TimeToPositionEvaluation::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }
}