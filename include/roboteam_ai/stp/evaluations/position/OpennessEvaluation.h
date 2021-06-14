//
// Created by maxl on 19-03-21.
//

#ifndef RTT_OPENNESSEVALUATION_H
#define RTT_OPENNESSEVALUATION_H

#include <NFParam/Param.h>
#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::evaluation {

    class OpennessEvaluation {
    public:
        OpennessEvaluation() noexcept;

        /**
         * Returns a uint8_t score linked to a position for its openness taken in account all enemy bots
         * @param enemyDistances Vector of all enemy distances from the point
         * @return uint8_t score
         */
        [[nodiscard]] uint8_t metricCheck(std::vector<double>& enemyDistances) const noexcept;

        const char* getName() { return "PositionOpenness"; }

    private:
        /**
         * Calculates the actual metric value using the piecewise linear function member
         * @param x the x of the function
         * @return metric value between 0-255
         */
        [[nodiscard]] uint8_t calculateMetric(const double& x) const noexcept;

        /**
         * Unique pointer to the piecewise linear function that calculates the fuzzy value
         */
        std::unique_ptr<nativeformat::param::Param> piecewiseLinearFunction;
    };

}  // namespace rtt::ai::stp::evaluation


#endif //RTT_OPENNESSEVALUATION_H
