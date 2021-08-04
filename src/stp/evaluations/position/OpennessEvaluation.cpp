//
// Created by maxl on 19-03-21.
//

#include <string>
#include "stp/evaluations/position/OpennessEvaluation.h"

namespace rtt::ai::stp::evaluation {
OpennessEvaluation::OpennessEvaluation() noexcept {
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

uint8_t OpennessEvaluation::metricCheck(std::vector<double>& enemyDistances) const noexcept {
    double evalScore = 0.0;
    /**
     * Limit the ranges from 2.5 to 0.5. [y = 1/x - 0.4]
     * (0,1.6)  |XX
     *          |  X
     *          |   X
     *          |    XX
     *          |      XXX
     *   (0,0)  |---------XXXXXX
     *              (Distance from Position)
     */
    for (auto& distance : enemyDistances){
        evalScore += 1/((distance < 0.5) ? 0.5 : (distance > 2.5) ? 2.5 : distance)-0.4;
    }
    return calculateMetric(evalScore);
}

uint8_t OpennessEvaluation::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }
}