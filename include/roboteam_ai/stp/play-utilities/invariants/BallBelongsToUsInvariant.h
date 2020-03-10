//
// Created by jessevw on 06.12.19.
//

#ifndef RTT_BALLBELONGSTOUSINVARIANT_H
#define RTT_BALLBELONGSTOUSINVARIANT_H

#include <include/roboteam_ai/world/Field.h>
#include <include/roboteam_ai/world_new/views/WorldDataView.hpp>

namespace rtt::ai::analysis {

/**
 * Invariant that is true when the ball belongs to us
 */
class BallBelongsToUsInvariant {
   public:
    /**
     * Functional implementation of when the ball belongs to us
     * @param world the current world state
     * @param field the current field state
     * @return true if the ball belongs to us, false otherwise
     */
    static bool isValid(world_new::view::WorldDataView world, const world::Field &field);
};
}  // namespace rtt::ai::analysis

#endif  // RTT_BALLBELONGSTOUSINVARIANT_H
