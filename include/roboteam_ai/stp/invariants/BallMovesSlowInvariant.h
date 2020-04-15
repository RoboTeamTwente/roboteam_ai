//
// Created by timovdk on 4/3/20.
//

#ifndef RTT_BALLMOVESSLOWINVARIANT_H
#define RTT_BALLMOVESSLOWINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class BallMovesSlowInvariant : public BaseInvariant {
   public:
    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const Field *field) const noexcept override;
   private:
    [[nodiscard]] uint8_t calculateMetric(const double& x) const noexcept;
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BALLMOVESSLOWINVARIANT_H
