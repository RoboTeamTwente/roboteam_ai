//
// Created by jordi on 19-05-20.
//

#ifndef RTT_BALLSHOTORCLOSETOTHEMINVARIANT_H
#define RTT_BALLSHOTORCLOSETOTHEMINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {

class BallShotOrCloseToThemInvariant : public BaseInvariant {
   public:
    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept override;

    const char* getName() override { return "BallShotOrCloseToThem"; }
};

}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BALLSHOTORCLOSETOTHEMINVARIANT_H
