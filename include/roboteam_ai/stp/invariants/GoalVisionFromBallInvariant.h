//
// Created by jordi on 12-05-20.
//

#ifndef RTT_GOALVISIONFROMBALLINVARIANT_H
#define RTT_GOALVISIONFROMBALLINVARIANT_H

#include <NFParam/Param.h>

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
/**
    Calculates goal vision percentage based on the position of the ball
**/
class GoalVisionFromBallInvariant : public BaseInvariant {
public:
    GoalVisionFromBallInvariant() noexcept;

    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept override;

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
    std::shared_ptr<nativeformat::param::Param> piecewiseLinearFunction;
};

}  // namespace rtt::ai::stp::invariant

#endif //RTT_GOALVISIONFROMBALLINVARIANT_H
