//
// Created by maxl on 24-03-21.
//

#ifndef RTT_TIMETOPOSITIONEVALUATION_H
#define RTT_TIMETOPOSITIONEVALUATION_H

#include <NFParam/Param.h>
#include "stp/constants/ControlConstants.h"
#include "include/roboteam_ai/world/views/RobotView.hpp"

namespace rtt::ai::stp::evaluation {

    class TimeToPositionEvaluation {
    public:
        TimeToPositionEvaluation() noexcept;

        /**
         * Compares the time for 2 robots to get to a point
         * @param robot1 First robot to compare with
         * @param robot2 Second robot to compare with
         * @param point Position that needs to be considered
         * @return a score based on the ratio between the 2 robots, TRUE is for the first robot being first
         */
        [[nodiscard]] uint8_t metricCheck(std::optional<world::view::RobotView> robot1, std::optional<world::view::RobotView> robot2, Vector2 point) const noexcept;

        const char* getName() { return "PositionTimeCompareToPoint"; }

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


#endif //RTT_TIMETOPOSITIONEVALUATION_H
