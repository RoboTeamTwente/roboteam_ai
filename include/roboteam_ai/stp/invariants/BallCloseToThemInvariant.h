//
// Created by jordi on 06-05-20.
//

#ifndef RTT_BALLCLOSETOTHEMINVARIANT_H
#define RTT_BALLCLOSETOTHEMINVARIANT_H

#include <NFParam/Param.h>
#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {

class BallCloseToThemInvariant : public BaseInvariant {
public:
    BallCloseToThemInvariant() noexcept;

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

#endif //RTT_BALLCLOSETOTHEMINVARIANT_H
