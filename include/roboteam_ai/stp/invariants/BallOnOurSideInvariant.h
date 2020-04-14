//
// Created by timovdk on 4/14/20.
//

#ifndef RTT_BALLONOURSIDEINVARIANT_H
#define RTT_BALLONOURSIDEINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class BallOnOurSideInvariant : public BaseInvariant {
    [[nodiscard]] bool checkInvariant(world_new::view::WorldDataView world, const Field *field) const noexcept override;
};
}


#endif  // RTT_BALLONOURSIDEINVARIANT_H
