//
// Created by maxl on 29-03-21.
//

#include <cmath>
#include <roboteam_utils/Vector2.h>
#include "stp/evaluations/position/BlockingEvaluation.h"

namespace rtt::ai::stp::evaluation {
BlockingEvaluation::BlockingEvaluation() noexcept {
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
    piecewiseLinearFunction = nativeformat::param::createParam(control_constants::FUZZY_FALSE, control_constants::FUZZY_TRUE, control_constants::FUZZY_FALSE, "positionOpenness");
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, 0.0);
    piecewiseLinearFunction->linearRampToYAtX(control_constants::FUZZY_FALSE, 2.5);
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_FALSE, 2.5);
}

uint8_t BlockingEvaluation::metricCheck(double pointDistance, std::vector<double>& enemyDistances, std::vector<double>& enemyAngles) const noexcept {
    double evalScore = 0.0;
    /**
     * Higher score for infront and further away
     */
     //TODO improve this evaluation
    for (int i = 0; i < enemyAngles.size(); i++){
        if (enemyDistances[i] > pointDistance && std::abs(enemyAngles[i])<M_PI_2/3) { // 30 degrees
            evalScore += pow((M_PI_2/3/std::abs(enemyAngles[i])),2) * pow(enemyDistances[i],1/2);
        }
    }
    return calculateMetric(evalScore);
}

uint8_t BlockingEvaluation::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }
}