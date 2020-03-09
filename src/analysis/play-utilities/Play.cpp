//
// Created by jessevw on 06.12.19.
//

#include "analysis/play-utilities/Play.h"

#include "analysis/play-utilities/invariants/AlwaysFalseInvariant.h"
#include "analysis/play-utilities/invariants/BallBelongsToUsInvariant.h"
#include "analysis/play-utilities/invariants/BallOnOurSideInvariant.h"

namespace rtt::ai::analysis {
    std::shared_ptr<bt::BehaviorTree> Play::getTree() const noexcept { return tree; }

    bool Play::isValidPlay(rtt::ai::world::World *world) const noexcept {
        return false;
    }

    uint8_t Play::scorePlay(rtt::ai::world::World *world) const noexcept {
        return 0;
    }
}  // namespace rtt::ai::analysis