//
// Created by timovdk on 4/14/20.
//

#ifndef RTT_BALLCLOSETOUSINVARIANT_H
#define RTT_BALLCLOSETOUSINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class BallCloseToUsInvariant : public BaseInvariant {
   public:
    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const Field *field) const noexcept override;
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BALLCLOSETOUSINVARIANT_H
