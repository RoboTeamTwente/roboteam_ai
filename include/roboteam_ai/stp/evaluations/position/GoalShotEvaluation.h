//
// Created by maxl on 23-03-21.
//

#ifndef RTT_GOALSHOTEVALUATION_H
#define RTT_GOALSHOTEVALUATION_H

#include <NFParam/Param.h>
#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::evaluation {

    class GoalShotEvaluation {
    public:
        GoalShotEvaluation() noexcept;

        [[nodiscard]] uint8_t metricCheck(double vis, double dist, double angle) const noexcept;

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

#endif //RTT_GOALSHOTEVALUATION_H
