//
// Created by maxl on 19-03-21.
//

#include <cmath>
#include "include/roboteam_ai/stp/evaluations/position/LineOfSightEvaluation.h"

namespace rtt::ai::stp::evaluation {
    LineOfSightEvaluation::LineOfSightEvaluation() noexcept {
        /**
         * Creates a piecewise linear function that looks as follows:
         *
         * (0,255)  |XXX
         *          |   XXX
         *          |      XXX
         *          |         XXX
         *   (0,0)  |------------XXX
         *                 (evalScore)
         */
        piecewiseLinearFunction = nativeformat::param::createParam(control_constants::FUZZY_FALSE, control_constants::FUZZY_TRUE, control_constants::FUZZY_FALSE, "positionLineOfSight");
        piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, 0.0);
        piecewiseLinearFunction->linearRampToYAtX(control_constants::FUZZY_FALSE, 50.0);
        piecewiseLinearFunction->setYAtX(control_constants::FUZZY_FALSE, 50.0);
    }

    uint8_t LineOfSightEvaluation::metricCheck(double pDist, std::vector<double>& eDists, std::vector<double>& eAngles) const noexcept {
        double evalScore = 0;
        /**
         *      /\                                            The positions in side the areas are evaluated.
         *    /   \            __________-----------------|
         *  /     |------------                           |    1. If there is a robot close and in front of the ball (1),
         * B   1   ==================== 2================ T       OR the angle is near 0 (so in path) the position is
         *  \     |------------__________                 |       returned as BAD = 0.
         *    \   /                      -----------------|    2. If there is a robot in side (2), there angle is evaluated.
         *      \/
         */
        for (int i = 0; i < eAngles.size(); i++){
            if (eDists[i] < pDist && std::abs(eAngles[i])<M_PI_2/3) { // 30 degrees
                if (eDists[i] < control_constants::DISTANCE_TO_ROBOT_FAR || std::abs(eAngles[i])<M_PI_2/18){ // 5 degrees
                    return 0;
                } else {
                    evalScore += pow((M_PI_2/3/std::abs(eAngles[i])),2);
                }
            }
        }
        return calculateMetric(evalScore);
    }

    uint8_t LineOfSightEvaluation::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }
}