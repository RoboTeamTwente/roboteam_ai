//
// Created by jessevw on 11.12.19.
//

#ifndef RTT_BALLONOURSIDEINVARIANT_H
#define RTT_BALLONOURSIDEINVARIANT_H

#include "world_old/World.h"
#include "world_old/Field.h"
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
        static bool isValid(rtt::ai::world::World *world, rtt::ai::world::Field *field);

    };
}


#endif //RTT_BALLONOURSIDEINVARIANT_H
