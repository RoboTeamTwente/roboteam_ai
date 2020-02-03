//
// Created by jessevw on 06.12.19.
//

#ifndef RTT_BALLBELONGSTOUSINVARIANT_H
#define RTT_BALLBELONGSTOUSINVARIANT_H

#include <include/roboteam_ai/world/Field.h>
#include <include/roboteam_ai/world/World.h>

namespace rtt::ai::analysis {
    using namespace rtt::ai::world;

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
        static bool isValid(rtt::ai::world::World *world, const Field *field);
    };
}  // namespace rtt::ai::analysis

#endif  // RTT_BALLBELONGSTOUSINVARIANT_H
