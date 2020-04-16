//
// Created by timovdk on 4/3/20.
//

#ifndef RTT_BALLMOVESSLOWINVARIANT_H
#define RTT_BALLMOVESSLOWINVARIANT_H

#include <NFParam/Param.h>

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class BallMovesSlowInvariant : public BaseInvariant {
   public:
    BallMovesSlowInvariant() noexcept;

    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const Field* field) const noexcept override;

   private:
    /**
     * Calculates the actual metric value using a piecewise linear function
     * @param x the x of the function
     * @return metric value between 0-255
     */
    [[nodiscard]] uint8_t calculateMetric(const double& x) const noexcept;

    std::shared_ptr<nativeformat::param::Param> piecewiseLinearFunction;
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BALLMOVESSLOWINVARIANT_H
