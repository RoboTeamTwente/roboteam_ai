//
// Created by jessevw on 11.12.19.
//

#ifndef RTT_ALWAYSTRUEINVARIANT_H
#define RTT_ALWAYSTRUEINVARIANT_H


#include "world_old/Field.h"
#include "world_old/World.h"

namespace rtt::ai::analysis {
    /**
     * Invariant that is true when the ball belongs to us
     */
    class AlwaysTrueInvariant {
    public:
        /**
         * Functional implementation of when the ball belongs to us
         * @param world the current world state
         * @param field the current field state
         * @return true if the ball belongs to us, false otherwise
         */
        static bool isValid(rtt::ai::world::World *world, rtt::ai::world::Field *field);

    };
}


#endif //RTT_ALWAYSTRUEINVARIANT_H
