//
// Created by jessevw on 11.12.19.
//

#ifndef RTT_BALLONOURSIDEINVARIANT_H
#define RTT_BALLONOURSIDEINVARIANT_H

#include "world/World.h"
#include "analysis/PlaysObjects/Invariants/Invariant.h"
namespace rtt::ai::analysis {
    /**
     * Invariant that is true when the ball belongs to us
     */
    class BallOnOurSide : public Invariant {
    public:
        /**
         * Functional implementation of when the ball is on our side of the field
         * @param world the current world state
         * @param field
         * @return true if the ball belongs to us, false otherwise
         */
        bool isTrue(rtt::ai::world::World* world, rtt::ai::world::Field* field);
    };
}


#endif //RTT_BALLONOURSIDEINVARIANT_H
