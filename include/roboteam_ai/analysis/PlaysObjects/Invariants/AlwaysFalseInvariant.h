//
// Created by jessevw on 11.12.19.
//

#ifndef RTT_ALWAYSFALSEINVARIANT_H
#define RTT_ALWAYSFALSEINVARIANT_H


#include <include/roboteam_ai/world/World.h>
#include <include/roboteam_ai/world/Field.h>

namespace rtt::ai::analysis {
    /**
     * Invariant that is true when the ball belongs to us
     */
    class AlwaysFalseInvariant {
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


#endif //RTT_ALWAYSFALSEINVARIANT_H
