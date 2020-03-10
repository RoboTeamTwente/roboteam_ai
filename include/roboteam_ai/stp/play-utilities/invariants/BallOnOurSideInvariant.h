//
// Created by jessevw on 11.12.19.
//

#ifndef RTT_BALLONOURSIDEINVARIANT_H
#define RTT_BALLONOURSIDEINVARIANT_H

#include <include/roboteam_ai/world_new/views/WorldDataView.hpp>
#include "world/Field.h"

namespace rtt::ai::analysis {

/**
 * Invariant that is true when the ball belongs to us
 */
class BallOnOurSideInvariant {
   public:
    /**
     * Functional implementation of when the ball is on our side of the field
     * @param world the current world state
     * @param field the current field state
     * @return true if the ball belongs to us, false otherwise
     */
    static bool isValid(world_new::view::WorldDataView world, const world::Field &field);
};
}  // namespace rtt::ai::analysis

#endif  // RTT_BALLONOURSIDEINVARIANT_H
