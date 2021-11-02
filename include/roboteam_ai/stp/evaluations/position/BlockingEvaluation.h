//
// Created by maxl on 29-03-21.
//

#ifndef RTT_BLOCKINGEVALUATION_H
#define RTT_BLOCKINGEVALUATION_H

#include <NFParam/Param.h>
#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::evaluation {

    class BlockingEvaluation {
    public:
        BlockingEvaluation() noexcept;
        /**
         * Returns a uint8_t score linked to a position for its possibility to score a goal from it
         * @param goalVisibility % visibility of the goal (taking in account other robots)
         * @param goalDistance distance from point to goal
         * @param goalAngle angle from point to goal (0 is straight in front the middle)
         * @return uint8_t score
         */
        [[nodiscard]] uint8_t metricCheck(double pointDistance, std::vector<double>& enemyDistances, std::vector<double>& enemyAngles) const noexcept;

        const char* getName() { return "PositionBlocking"; }

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
#endif //RTT_BLOCKINGEVALUATION_H
