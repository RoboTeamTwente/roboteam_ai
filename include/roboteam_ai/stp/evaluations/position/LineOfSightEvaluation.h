//
// Created by maxl on 22-03-21.
//

#ifndef RTT_LINEOFSIGHTEVALUATION_H
#define RTT_LINEOFSIGHTEVALUATION_H

#include <NFParam/Param.h>
#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::evaluation {

    class LineOfSightEvaluation {
    public:
        LineOfSightEvaluation() noexcept;

        /**
         * Returns a uint8_t score linked to a position for each enemy robot on the line to the position from the ball
         * @param receiverDistance Receiver distance from ball
         * @param enemyDistances Vector of all enemy distances from ball
         * @param enemyAngles Vector of all enemy angels from ball compared to Receiver angle (= 0 degrees)
         * @return uint8_t score
         */
        [[nodiscard]] uint8_t metricCheck(double receiverDistance, std::vector<double>& enemyDistances, std::vector<double>& enemyAngles) const noexcept;

        const char* getName() { return "PositionLineOfSight"; }

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
#endif //RTT_LINEOFSIGHTEVALUATION_H
