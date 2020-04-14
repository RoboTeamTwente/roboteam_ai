//
// Created by ratoone on 27-03-20.
//

#ifndef RTT_HALTGAMESTATEINVARIANT_H
#define RTT_HALTGAMESTATEINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class HaltGameStateInvariant : public BaseInvariant {
   public:
    [[nodiscard]] double metricCheck(world_new::view::WorldDataView world, const Field *field) const noexcept override;
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_HALTGAMESTATEINVARIANT_H
